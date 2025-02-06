import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import torch
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Define the ActorCritic class to match the saved checkpoint
class ActorCritic(torch.nn.Module):
    def __init__(self, obs_dim, action_dim, actor_hidden_dims, critic_hidden_dims, activation='elu'):
        super(ActorCritic, self).__init__()
        # Activation function
        if activation == 'elu':
            self.activation = torch.nn.ELU()
        elif activation == 'relu':
            self.activation = torch.nn.ReLU()
        else:
            raise ValueError(f"Unsupported activation: {activation}")

        # Actor network
        actor_layers = []
        prev_dim = obs_dim
        for dim in actor_hidden_dims:
            actor_layers.append(torch.nn.Linear(prev_dim, dim))
            actor_layers.append(self.activation)
            prev_dim = dim
        actor_layers.append(torch.nn.Linear(prev_dim, action_dim))
        self.actor = torch.nn.Sequential(*actor_layers)

        # Critic network
        critic_layers = []
        prev_dim = obs_dim
        for dim in critic_hidden_dims:
            critic_layers.append(torch.nn.Linear(prev_dim, dim))
            critic_layers.append(self.activation)
            prev_dim = dim
        critic_layers.append(torch.nn.Linear(prev_dim, 1))
        self.critic = torch.nn.Sequential(*critic_layers)

    def forward(self, obs):
        action_logits = self.actor(obs)
        value = self.critic(obs)
        return action_logits, value


class MotorSRVNode(Node):
    def __init__(self):
        super().__init__('RL_implementation')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(Imu, '/camera/gyro/sample', self.gyro_callback, sensor_qos)
        self.create_subscription(Imu, '/camera/accel/sample', self.accel_callback, sensor_qos)
        self.create_subscription(Twist, '/motor_commands', self.command_callback, sensor_qos)
        self.create_subscription(Float32MultiArray, '/dynamixel_status', self.dynamixel_callback, sensor_qos)

        self.publisher = self.create_publisher(Float32MultiArray, '/robot/motor_commands', 10)

        self.base_ang_vel = torch.zeros(3)
        self.projected_gravity = torch.zeros(3)
        self.commands = torch.zeros(3)
        self.dof_pos = torch.zeros(8)
        self.dof_vel = torch.zeros(8)
        self.actions = torch.zeros(8)
        self.initial_gravity = None

        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

        # Load the policy model and configuration file
        self.policy = self.load_policy(
            "src/mingsong_turtle_try/src/motor_srv/motor_srv/model_800.pt",
            obs_dim=33,  # Observation dimension
            action_dim=8,  # Action dimension
            actor_hidden_dims=[512, 256, 128],
            critic_hidden_dims=[512, 256, 128],
            activation="elu",
        )

        self.default_dof_pos = torch.tensor(
            [1.3, -1.3, 1.3, -1.3, 0.15, -0.15, 0.15, -0.15], dtype=torch.float32
        ).to(self.device)

        self.create_timer(1.0, self.compute_and_publish)

    def load_policy(self, checkpoint_path, obs_dim, action_dim, actor_hidden_dims, critic_hidden_dims, activation):
        """Load the policy network from a checkpoint file."""
        policy_net = ActorCritic(
            obs_dim=obs_dim,
            action_dim=action_dim,
            actor_hidden_dims=actor_hidden_dims,
            critic_hidden_dims=critic_hidden_dims,
            activation=activation,
        ).to(self.device)

        # Load the checkpoint
        checkpoint = torch.load(checkpoint_path, map_location=self.device)

        # Load the model weights
        policy_net.load_state_dict(checkpoint["model_state_dict"])  # Adjust key if necessary
        policy_net.eval()

        self.get_logger().info("Policy model loaded successfully.")
        return policy_net.actor  # Return only the Actor network for inference

    def encoder_to_rad(self, encoder_value):
        return (encoder_value - 2048) * (np.pi / 2048)

    def rad_to_encoder(self, rad_value):
        return int((rad_value / np.pi * 2048) + 2048)

    def rotate_vector(self, vector, rotation_matrix):
        return np.dot(rotation_matrix, vector)

    def gyro_callback(self, msg):
        raw_ang_vel = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        rotation_z = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        rotation_x = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
        rotated_ang_vel = self.rotate_vector(self.rotate_vector(raw_ang_vel, rotation_z), rotation_x)
        self.base_ang_vel = torch.tensor(rotated_ang_vel, dtype=torch.float32)

    def accel_callback(self, msg):
        raw_accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        rotation_z = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        rotation_x = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
        rotated_accel = self.rotate_vector(self.rotate_vector(raw_accel, rotation_z), rotation_x)
        norm = np.linalg.norm(rotated_accel)
        if norm > 0:
            rotated_accel /= norm
        if self.initial_gravity is None:
            self.initial_gravity = rotated_accel
        self.projected_gravity = torch.tensor(rotated_accel - self.initial_gravity, dtype=torch.float32)

    def command_callback(self, msg):
        self.commands = torch.tensor([msg.linear.x, msg.linear.y, msg.angular.z])

    def dynamixel_callback(self, msg):
        indices = [1, 4, 7, 10, 13, 16, 19, 22]
        raw_positions = [msg.data[i] for i in indices]
        self.dof_pos = torch.tensor([self.encoder_to_rad(pos) for pos in raw_positions])
        raw_velocities = [msg.data[i + 1] for i in indices]
        self.dof_vel = torch.tensor([(vel * 0.229 * (2 * np.pi / 60)) for vel in raw_velocities])

    def compute_and_publish(self):
    # Construct the observation buffer
        obs_buf = torch.cat([
            self.base_ang_vel * 0.25,
            self.projected_gravity,
            self.commands * 1,
            (self.dof_pos - self.default_dof_pos) * 1,
            self.dof_vel * 0.05,
            self.actions,
        ], axis=-1).to(self.device)

        # Print or log the observation buffer for debugging
        self.get_logger().info(f'Observation Buffer: {obs_buf.tolist()}')  # ROS logger
        # Alternatively, use a standard print statement (not recommended for ROS)
        # print(f'Observation Buffer: {obs_buf.tolist()}')

        # Use the policy network to compute actions
        with torch.no_grad():
            self.actions = self.policy(obs_buf.unsqueeze(0)).squeeze(0).cpu()

        # Convert action outputs (radians) back to encoder values
        encoder_commands = [self.rad_to_encoder(action) for action in self.actions.tolist()]
        msg = Float32MultiArray()
        msg.data = encoder_commands

        # Publish the commands and log the output
        self.publisher.publish(msg)
        self.get_logger().info(f'Published actions (encoder values): {encoder_commands}')



def main(args=None):
    rclpy.init(args=args)
    node = MotorSRVNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
import time
import torch
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from dynamixel_sdk import *
from dynamixel_sdk.registerDict import X_Series
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class ActorCritic(torch.nn.Module):
    def __init__(self, obs_dim, action_dim, actor_hidden_dims, critic_hidden_dims, activation='elu'):
        super(ActorCritic, self).__init__()
        if activation == 'elu':
            self.activation = torch.nn.ELU()

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


class UnifiedDynamixelRLNode(Node):
    def __init__(self):
        super().__init__('RL_new')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to IMU topics
        self.create_subscription(Imu, '/camera/gyro/sample', self.gyro_callback, sensor_qos)
        self.create_subscription(Imu, '/camera/accel/sample', self.accel_callback, sensor_qos)
        self.create_subscription(Twist, '/motor_commands', self.command_callback, sensor_qos)

        # Dynamixel settings
        self.DEVICENAME = '/dev/ttyUSB0'
        self.BAUDRATE = 4000000
        self.PROTOCOL_VERSION = 2.0
        self.DXL_IDS = list([0,1,2,3,4,5,6,7,8,9,10,11])  # Motor IDs from 0 to 11
        self.ACTIVE_DXL_IDS = [0, 1, 2, 3, 4, 5, 6, 7]  # Only motors 0-7 are active
        self.FIXED_DXL_IDS = [8, 9, 10, 11]  # Motors 8-11 always set to 2048

        # Initial motor positions
        self.goal_positions = [1205, 2890, 1205, 2890, 1956, 2147, 1956, 2147]
        self.target_positions = self.goal_positions[:]
        self.fixed_positions = [2048] * 4  # Motors 8-11 fixed

        # Initialize communication with Dynamixel motors
        self.port_handler = PortHandler(self.DEVICENAME)
        self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            self.get_logger().error("Failed to open port")
            rclpy.shutdown()
            return

        if not self.port_handler.setBaudRate(self.BAUDRATE):
            self.get_logger().error("Failed to set baudrate")
            rclpy.shutdown()
            return

        self.get_logger().info("Port opened and baudrate set")

        # Initialize SyncWrite
        self.goal_position_write = GroupSyncWrite(self.port_handler, self.packet_handler, 
                                                  X_Series["ADDR_GOAL_POSITION"], X_Series["LEN_GOAL_POSITION"])

        self.enable_torque()
        self.set_motor_gains(kp=300, kd=0)
        self.move_to_initial_positions()

        # SyncRead for position and velocity
        self.position_read = GroupSyncRead(self.port_handler, self.packet_handler, 
                                           X_Series["ADDR_PRESENT_POSITION"], X_Series["LEN_PRESENT_POSITION"])
        self.velocity_read = GroupSyncRead(self.port_handler, self.packet_handler, 
                                           X_Series["ADDR_PRESENT_VELOCITY"], X_Series["LEN_PRESENT_VELOCITY"])

        for dxl_id in self.ACTIVE_DXL_IDS:
            self.position_read.addParam(dxl_id)
            self.velocity_read.addParam(dxl_id)
        

        # RL Model
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.policy = self.load_policy(
            "src/mingsong_turtle_try/src/motor_srv/motor_srv/model_700.pt",
            obs_dim=33, action_dim=8, 
            actor_hidden_dims=[512, 256, 128], 
            critic_hidden_dims=[512, 256, 128], 
            activation="elu"
        )

        # Motion Parameters
        self.base_ang_vel = torch.zeros(3, device=self.device)
        self.projected_gravity = torch.zeros(3, device=self.device)
        self.commands = torch.zeros(3, device=self.device)
        self.dof_pos = torch.zeros(8, device=self.device)
        self.dof_vel = torch.zeros(8, device=self.device)
        self.actions = torch.zeros(8, device=self.device)
        self.default_dof_pos = torch.tensor(
            [1.3, -1.3, 1.3, -1.3, 0.15, -0.15, 0.15, -0.15], dtype=torch.float32
        ).to(self.device)

        # self.timer = self.create_timer(1.0, self.start_control_loop)
        self.create_timer(0.02, self.control_loop)

    def enable_torque(self):
        for dxl_id in self.DXL_IDS:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, dxl_id, X_Series["ADDR_TORQUE_ENABLE"], X_Series["TORQUE_ENABLE"]
            )
            if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
                self.get_logger().info(f"Motor {dxl_id} torque enabled successfully")
            else:
                self.get_logger().error(f"Failed to enable torque for Motor {dxl_id}, error: {dxl_comm_result}")

    def set_motor_gains(self, kp=640, kd=0):
        """
        Sets the Kp and Kd values for all motors.

        :param kp: Proportional gain (default = 640)
        :param kd: Derivative gain (default = 0)
        """
        kp = int(kp)  # Convert to integer
        kd = int(kd)  # Convert to integer

        for dxl_id in self.DXL_IDS:
            # Ensure the motor is in position control mode
            operating_mode, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(
                self.port_handler, dxl_id, 11  # Address 11: Operating Mode
            )

            if operating_mode not in [3, 4]:  # Check if in Position Control Mode
                self.get_logger().error(f"Motor {dxl_id} is not in Position Control Mode! Current mode: {operating_mode}")
                continue  # Skip this motor if not in the correct mode

            # Set Kp (Proportional Gain)
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
                self.port_handler, dxl_id, 84, kp  # Address 84: Position P Gain
            )
            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                self.get_logger().error(f"Failed to set Kp for Motor {dxl_id}, error: {dxl_comm_result}")

            # Set Kd (Derivative Gain)
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
                self.port_handler, dxl_id, 80, kd  # Address 80: Position D Gain
            )
            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                self.get_logger().error(f"Failed to set Kd for Motor {dxl_id}, error: {dxl_comm_result}")

        self.get_logger().info(f"Set Kp={kp}, Kd={kd} for all motors.")


    def move_to_initial_positions(self):
        for dxl_id, pos in zip(self.ACTIVE_DXL_IDS, self.goal_positions):
            self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, X_Series["ADDR_GOAL_POSITION"], pos)

        for dxl_id in self.FIXED_DXL_IDS:
            self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, X_Series["ADDR_GOAL_POSITION"], 2048)

        self.get_logger().info("Motors initialized. Waiting 1s...")
        time.sleep(1)

    def load_policy(self, checkpoint_path, obs_dim, action_dim, actor_hidden_dims, critic_hidden_dims, activation):
        """Load only the actor (student) policy network from a checkpoint file."""
        # Instantiate the full ActorCritic network (we only need the actor part later)
        policy_net = ActorCritic(
            obs_dim=obs_dim,
            action_dim=action_dim,
            actor_hidden_dims=actor_hidden_dims,
            critic_hidden_dims=critic_hidden_dims,
            activation=activation,
        ).to(self.device)
        # Load the checkpoint
        checkpoint = torch.load(checkpoint_path, map_location=self.device)
        state_dict = checkpoint["model_state_dict"]
        # Filter out only the actor parameters.
        # This keeps only keys that start with "actor." and removes the prefix so they match the actor's own keys.
        actor_state_dict = {k.replace("actor.", ""): v for k, v in state_dict.items() if k.startswith("actor.")}
        # Load the filtered state dict into the actor network
        policy_net.actor.load_state_dict(actor_state_dict, strict=False)
        policy_net.actor.eval()  # Set the actor network to evaluation mode.
        self.get_logger().info("Policy actor loaded successfully.")
        return policy_net.actor  # Return only the actor network for inference.
    
    def encoder_to_rad(self, encoder_value):
    # Maps: encoder=2048 -> rad=0, 0 -> +pi, 4096 -> -pi
        return -(np.pi / 2048.0) * (encoder_value - 2048)


    def rad_to_encoder(self, rad_value):
    # Inverse of the above; ensures 0->2048, +pi->0, -pi->4096
        return int(
            2048 - (2048.0 * rad_value / np.pi)
        )


    def rotate_vector(self, vector, rotation_matrix):
        return np.dot(rotation_matrix, vector)
    
    def command_callback(self, msg):
            self.commands = torch.tensor([msg.linear.x, msg.linear.y, msg.angular.z], device=self.device)

    def gyro_callback(self, msg):
        raw_ang_vel = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        R = np.array([
            [0,  0,  -1],   
            [-1, 0,  0],
            [0, 1,  0]
        ])
        rotated_ang_vel = self.rotate_vector(raw_ang_vel, R)
        self.base_ang_vel = torch.tensor(rotated_ang_vel, dtype=torch.float32, device=self.device)

    def accel_callback(self, msg):
    # 1) Extract raw acceleration from IMU message
        raw_accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        #    - IMU z -> Robot x
        #    - IMU y -> Robot -z
        #    - IMU x -> Robot -y
        R = np.array([
            [0,  0,  -1],   
            [-1, 0,  0],
            [0, 1,  0]
        ])

        rotated_accel = self.rotate_vector(raw_accel, R)

        norm = np.linalg.norm(rotated_accel)
        if norm > 0:
            rotated_accel /= norm

        self.projected_gravity = torch.tensor(rotated_accel, dtype=torch.float32, device=self.device)

    def convert_to_signed(self, value, bit_length=32):
        if value >= (1 << (bit_length - 1)):
            value -= (1 << bit_length)
        return value

    # def start_control_loop(self):
    #     self.create_timer(0.06, self.control_loop)

    def control_loop(self):
        # Read positions and velocities from Dynamixel motors
        self.position_read.txRxPacket()
        self.velocity_read.txRxPacket()

        raw_positions = []
        raw_velocities = []

        for dxl_id in self.ACTIVE_DXL_IDS:
            # Get position (4 bytes, needs conversion)
            pos = self.position_read.getData(dxl_id, X_Series["ADDR_PRESENT_POSITION"], 4)
            vel = self.velocity_read.getData(dxl_id, X_Series["ADDR_PRESENT_VELOCITY"], 4)

            # Convert velocity from unsigned to signed
            vel = self.convert_to_signed(vel, 32)

            raw_positions.append(pos)
            raw_velocities.append(-vel)
            print(raw_positions)
            print(raw_velocities)

        # Convert encoder values to radians
        self.dof_pos = torch.tensor([self.encoder_to_rad(pos) for pos in raw_positions], device=self.device)

        # Convert velocities to rad/s
        self.dof_vel = torch.tensor([(vel * 0.229 * (2 * torch.pi / 60)) for vel in raw_velocities], device=self.device)

        # print(f"Updated dof_pos: {self.dof_pos.tolist()}")
        # print(f"Updated dof_vel: {self.dof_vel.tolist()}")
        obs_buf = torch.cat([
            self.base_ang_vel * 0.25,
            self.projected_gravity,
            self.commands,
            (self.dof_pos - self.default_dof_pos).clamp(-0.7, 0.7),
            self.dof_vel * 0.05,
            self.actions,
        ], axis=-1).to(self.device)


        #self.get_logger().info(f'Observation Buffer: {obs_buf.tolist()}')

        with torch.no_grad():
            self.actions = self.policy(obs_buf.unsqueeze(0)).squeeze(0)

        adjusted_actions = (self.actions * 0.25) + self.dof_pos
        encoder_commands = [self.rad_to_encoder(action) for action in adjusted_actions.cpu().tolist()]

        #self.get_logger().info(f'Published actions (encoder values): {adjusted_actions}')

        for dxl_id, command in zip(self.ACTIVE_DXL_IDS, encoder_commands):
            self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, X_Series["ADDR_GOAL_POSITION"], command)

        for dxl_id in self.FIXED_DXL_IDS:
            self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, X_Series["ADDR_GOAL_POSITION"], 2048)


def main():
    rclpy.init()
    node = UnifiedDynamixelRLNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

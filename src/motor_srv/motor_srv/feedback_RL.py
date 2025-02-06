import rclpy
from rclpy.node import Node
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import torch
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class MotorSRVNode(Node):
    def __init__(self):
        super().__init__('feedback_RL')
        
        # Define compatible QoS for sensor topics
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.create_subscription(Imu, '/camera/gyro/sample', self.gyro_callback, sensor_qos)
        self.create_subscription(Imu, '/camera/accel/sample', self.accel_callback, sensor_qos)
        self.create_subscription(Twist, '/motor_commands', self.command_callback, sensor_qos)
        self.create_subscription(Float32MultiArray, '/dynamixel_status', self.dynamixel_callback, sensor_qos)

        # Initialize variables
        self.base_ang_vel = torch.zeros(3)
        self.projected_gravity = torch.zeros(3)
        self.commands = torch.zeros(3)
        self.dof_pos = torch.zeros(8)
        self.dof_vel = torch.zeros(8)
        self.actions = torch.zeros(8)
        self.initial_gravity = None

        # Scaling factors (example values)
        self.obs_scales = {"ang_vel": 1.0, "dof_pos": 1.0, "dof_vel": 1.0}
        self.commands_scale = 1.0
        self.default_dof_pos = torch.zeros(8)

        # Timer to compute observation buffer periodically
        self.create_timer(1.0, self.compute_obs_buf)
    
    def gyro_callback(self, msg: Imu):
        self.base_ang_vel = torch.tensor([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
    
    def accel_callback(self, msg: Imu):
        raw_accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        if self.initial_gravity is None:
            self.initial_gravity = raw_accel
            #self.get_logger().info(f"Initial gravity vector recorded: {self.initial_gravity}")
        
        self.projected_gravity = torch.tensor(raw_accel - self.initial_gravity)
        #self.get_logger().info(f"Gravity Compensated Acceleration: {self.projected_gravity}")
    
    def command_callback(self, msg: Twist):
        self.commands = torch.tensor([
            msg.linear.x,
            msg.linear.y,
            msg.angular.z
        ])
        print("/motor_commands:", self.commands.numpy())
    
    def dynamixel_callback(self, msg: Float32MultiArray):
        selected_indices = [1, 4, 7, 10, 13, 16, 19, 22]  # Positions for motors 0,1,3,4,6,7,9,10
        self.dof_pos = torch.tensor([msg.data[i] for i in selected_indices])

        velocity_indices = [i + 1 for i in selected_indices]  # Velocities
        self.dof_vel = torch.tensor([msg.data[i] for i in velocity_indices])
        # print("dof_pos:", self.dof_pos.numpy())
        # print("dof_vel:", self.dof_vel.numpy())
    
    def compute_obs_buf(self):
        obs_buf = torch.cat([
            self.base_ang_vel * self.obs_scales["ang_vel"],
            self.projected_gravity,
            self.commands * self.commands_scale,
            (self.dof_pos - self.default_dof_pos) * self.obs_scales["dof_pos"],
            self.dof_vel * self.obs_scales["dof_vel"],
            self.actions,
        ], axis=-1)
        self.get_logger().info(f'Observation buffer: {obs_buf}')


def main(args=None):
    rclpy.init(args=args)
    node = MotorSRVNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

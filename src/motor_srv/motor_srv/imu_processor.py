import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
from visualization_msgs.msg import Marker
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from filterpy.kalman import KalmanFilter

class ImuProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_tf)

        # Define compatible QoS for sensor topics
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers for IMU data
        self.gyro_sub = self.create_subscription(Imu, '/camera/gyro/sample', self.gyro_callback, sensor_qos)
        self.accel_sub = self.create_subscription(Imu, '/camera/accel/sample', self.accel_callback, sensor_qos)

        # Publishers
        self.odom_publisher_ = self.create_publisher(Odometry, 'imu_odometry', 10)
        self.marker_publisher_ = self.create_publisher(Marker, 'imu_velocity_marker', 10)

        # Data storage for acceleration and gyroscope readings
        self.linear_accel = np.zeros(3)
        self.angular_velocity = np.zeros(3)

        # Gravity estimation variable
        self.initial_gravity = None

        # Time tracking for integration using ROS 2 time
        self.last_time = None
        self.velocity = np.zeros(3)

        # # Initialize Kalman Filter
        # self.kf = KalmanFilter(dim_x=3, dim_z=3)
        # self.kf.F = np.eye(3)  # State transition model (constant velocity)
        # self.kf.H = np.eye(3)  # Measurement model
        # self.kf.P *= 100  # Initial state covariance
        # self.kf.R = np.eye(3) * 0.01  # Measurement noise
        # self.kf.Q = np.eye(3) * 0.01  # Process noise
        # self.kf.x = np.zeros((3, 1))  # Initial velocity state

        self.get_logger().info("IMU Processor Node Initialized with BEST_EFFORT QoS")

    def accel_callback(self, msg):
        raw_accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # Capture the first gravity vector reading
        if self.initial_gravity is None:
            self.initial_gravity = raw_accel
            self.get_logger().info(f"Initial gravity vector recorded: {self.initial_gravity}")

        # Subtract the initial gravity vector for gravity compensation
        self.linear_accel = raw_accel - self.initial_gravity

        self.get_logger().info(f"Raw Acceleration: {raw_accel}")
        self.get_logger().info(f"Gravity Compensated Acceleration: {self.linear_accel}")
        self.process_data()

    def gyro_callback(self, msg):
        self.angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        self.process_data()

    def process_data(self):
    # Get current time from ROS 2 clock in seconds
        current_time = self.get_clock().now().nanoseconds / 1e9

        if self.last_time is None or self.initial_gravity is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        self.last_time = current_time

        # Kalman filter prediction and update (Remove or comment out)
        # self.kf.predict()
        # accel_measurement = self.linear_accel.reshape((3, 1)) * dt
        # self.kf.update(accel_measurement)
        # self.velocity = self.kf.x.flatten()

        # Replace with direct integration (v = u + at)
        self.velocity += self.linear_accel * dt

        # Apply velocity damping to counter drift
        if np.linalg.norm(self.linear_accel) < 0.1:  # Noise threshold
            self.velocity *= 0.98  # Apply small damping to reduce drift

        # Publish odometry message for visualization
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.twist.twist.linear.x = self.velocity[0]
        odom_msg.twist.twist.linear.y = self.velocity[1]
        odom_msg.twist.twist.linear.z = self.velocity[2]

        self.odom_publisher_.publish(odom_msg)

        # Publish velocity as Marker for RViz
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.scale.x = max(np.linalg.norm(self.velocity) * 1, 0.1)  # Arrow length scaled by velocity
        marker.scale.y = 0.05  # Arrow width
        marker.scale.z = 0.05  # Arrow height

        marker.color.a = 1.0
        marker.color.r = 1.0  # Red color for velocity

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.pose.orientation.x = self.velocity[0]
        marker.pose.orientation.y = self.velocity[1]
        marker.pose.orientation.z = self.velocity[2]
        marker.pose.orientation.w = 1.0  # Assuming unity orientation

        self.marker_publisher_.publish(marker)

        # Debugging output
        self.get_logger().info(f"Computed Linear Velocity: {self.velocity}")


    def broadcast_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ImuProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class ImuRgbdFusionNode(Node):
    def __init__(self):
        super().__init__('imu_rgbd_fused')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_tf)

        # Define QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers for IMU and RGB-D data
        self.gyro_sub = self.create_subscription(Imu, '/camera/gyro/sample', self.gyro_callback, sensor_qos)
        self.accel_sub = self.create_subscription(Imu, '/camera/accel/sample', self.accel_callback, sensor_qos)
        self.rgb_sub = self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, sensor_qos)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, sensor_qos)

        # Publishers for velocity visualization
        self.odom_publisher_ = self.create_publisher(Odometry, 'imu_rgbd_odometry', 10)
        self.marker_publisher_ = self.create_publisher(Marker, 'imu_rgbd_velocity_marker', 10)

        # IMU data storage
        self.linear_accel = np.zeros(3)
        self.angular_velocity = np.zeros(3)

        # Gravity estimation
        self.initial_gravity = None

        # Time tracking
        self.last_time = None
        self.velocity = np.zeros(3)

        # RGB-D processing
        self.bridge = CvBridge()
        self.prev_gray = None

        self.get_logger().info("IMU RGB-D Fusion Node Initialized")

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

        # Subtract the gravity vector
        self.linear_accel = raw_accel - self.initial_gravity
        self.process_data()

    def gyro_callback(self, msg):
        self.angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        self.process_data()

    def rgb_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_rgb(cv_image)

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

    def process_data(self):
        # Get current time from ROS 2 clock
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.last_time is None or self.initial_gravity is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        self.last_time = current_time

        # Integrate acceleration for velocity estimate
        self.velocity += self.linear_accel * dt

        # Apply velocity damping
        if np.linalg.norm(self.linear_accel) < 0.1:
            self.velocity *= 0.98

    def process_rgb(self, cv_image):
        """Process RGB image to compute optical flow"""
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        if self.prev_gray is not None:
            flow = cv2.calcOpticalFlowFarneback(self.prev_gray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
            avg_flow = np.mean(flow, axis=(0, 1))

            # Convert pixel flow to world velocity (simple scaling)
            flow_velocity = np.array([avg_flow[0] * 0.01, avg_flow[1] * 0.01, 0.0])

            # Fuse IMU and optical flow velocity (weighted average)
            self.velocity = 0.5 * self.velocity + 0.5 * flow_velocity

        self.prev_gray = gray
        self.publish_velocity()

    def publish_velocity(self):
        # Publish odometry message
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

        marker.scale.x = max(np.linalg.norm(self.velocity) * 1, 0.1)
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.a = 1.0
        marker.color.g = 1.0  # Green for fused velocity

        marker.pose.orientation.x = self.velocity[0]
        marker.pose.orientation.y = self.velocity[1]
        marker.pose.orientation.z = self.velocity[2]
        marker.pose.orientation.w = 1.0

        self.marker_publisher_.publish(marker)

        self.get_logger().info(f"Fused Velocity: {self.velocity}")

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
    node = ImuRgbdFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

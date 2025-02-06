import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ImuRgbdPublisher(Node):
    def __init__(self):
        super().__init__('imu_rgbd_publisher')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.odom_pub = self.create_publisher(Odometry, '/camera/odom/sample', 10)

        self.timer = self.create_timer(0.1, self.publish_sensor_data)
        self.get_logger().info('IMU and Odometry Publisher Initialized')

    def publish_sensor_data(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81
        self.imu_pub.publish(imu_msg)

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.twist.twist.linear.x = 0.5
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        self.odom_pub.publish(odom_msg)

        self.get_logger().info('Published IMU and Odometry data')

def main(args=None):
    rclpy.init(args=args)
    node = ImuRgbdPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

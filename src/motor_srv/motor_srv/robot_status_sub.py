import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import PoseStamped
from rosbag2_py import SequentialWriter
import math
import time
import rosbag2_py
from rclpy.serialization import serialize_message
from datetime import datetime
import os

class RobotStatusSubscriber(Node):

    def __init__(self):
        super().__init__('robot_status_subscriber')
        self.subscription = self.create_subscription(
            Bool,
            '/robot_status',
            self.status_callback,
            10
        )
        self.recording = False
        self.recording_start_time = None
        self.total_recording_time = 0
        self.power_data = []
        self.total_energy = 0.0
        self.data_collect_time_gap = 0.1

        self.power_subscription = self.create_subscription(
            Float32,
            '/power',
            self.power_callback,
            10
        )

        self.pose_data = []
        self.total_distance = 0.0

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/visual_slam/tracking/vo_pose',
            self.pose_callback,
            10
        )

        self.cot_publisher = self.create_publisher(Float32, '/cot', 10)

        self.bag = None

    def status_callback(self, msg):
        if msg.data:
            self.get_logger().info('Robot is recording...')
            self.recording = True
            self.recording_start_time = time.time()

            # Generate a timestamp to the minute
            timestamp = datetime.now().strftime("%m-%d_%H-%M-%S")
            folder_path = "data/bags/"
            bag_name = os.path.join(folder_path, f'robot_data_{timestamp}.bag')

            # Create the data/bags folder if it doesn't exist
            os.makedirs(folder_path, exist_ok=True)

            # Create a rosbag with the timestamped name using rosbag2_py
            self.bag = SequentialWriter()
            storage_options = rosbag2_py._storage.StorageOptions(
                uri=bag_name,
                storage_id='sqlite3'
            )
            converter_options = rosbag2_py._storage.ConverterOptions('', '')
            self.bag.open(storage_options, converter_options)
            
            topic_info1 = rosbag2_py._storage.TopicMetadata(
                name='/power',
                type='std_msgs/msg/Float32',
                serialization_format='cdr')
            self.bag.create_topic(topic_info1)

            topic_info2 = rosbag2_py._storage.TopicMetadata(
                name='/visual_slam/tracking/vo_pose',
                type='geometry_msgs/msg/PoseStamped',
                serialization_format='cdr')
            self.bag.create_topic(topic_info2)


        else:
            self.get_logger().info('Robot is not recording...')
            if self.recording_start_time is not None:
                recording_end_time = time.time()
                recording_duration = recording_end_time - self.recording_start_time
                self.total_recording_time += recording_duration
                self.get_logger().info(f'Recording duration: {recording_duration} seconds')
                self.calculate_total_energy()
                self.calculate_total_distance()
                self.calculate_cot()
                self.recording = False
                self.power_data = []
                self.pose_data = []

                # Close the rosbag when recording ends
                # self.bag.close()
            self.recording_start_time = None

    def calculate_cot(self):
        if self.total_distance > 0:
            robot_mass = 7 #in kg. With shell is 7kg, without is 6.3kg. 
            cot = self.total_energy / (self.total_distance*9.8*robot_mass)
            self.get_logger().info(f'Cost of Transport (COT): {cot:.2f}')
            cot_msg = Float32()
            cot_msg.data = cot
            self.cot_publisher.publish(cot_msg)

    def power_callback(self, msg):
        if self.recording:
            power_value = msg.data
            self.power_data.append(power_value)
            # Save power data to the rosbag using rosbag2_py
            self.bag.write('/power', serialize_message(msg), self.get_clock().now().nanoseconds)

    def pose_callback(self, msg):
        if self.recording:
            self.pose_data.append(msg)
            # Save pose data to the rosbag using rosbag2_py
            self.bag.write('/visual_slam/tracking/vo_pose', serialize_message(msg), self.get_clock().now().nanoseconds)

    def calculate_total_energy(self):
        if not self.power_data:
            return

        total_energy = sum(self.power_data) * self.data_collect_time_gap
        self.total_energy = total_energy
        self.get_logger().info(f'Total Energy: {total_energy:.2f} Joules')

    def calculate_total_distance(self):
        if len(self.pose_data) >= 2:
            start_pose = self.pose_data[0]
            end_pose = self.pose_data[-1]
            dx = start_pose.pose.position.x - end_pose.pose.position.x
            dy = start_pose.pose.position.y - end_pose.pose.position.y
            dz = start_pose.pose.position.z - end_pose.pose.position.z
            total_distance = math.sqrt(dx**2 + dy**2 + dz**2)
            self.total_distance = total_distance
            self.get_logger().info(f'Total Distance: {total_distance:.2f} meters')

def main(args=None):
    rclpy.init(args=args)
    robot_status_subscriber = RobotStatusSubscriber()
    rclpy.spin(robot_status_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class SinusoidalGoalPublisher(Node):
    def __init__(self):
        super().__init__('sinusoidal_goal_publisher')

        self.publisher_ = self.create_publisher(Float32MultiArray, 'goal_positions', 10)
        self.UPDATE_INTERVAL = 0.01  # 100 Hz for smooth motion
        self.timer = self.create_timer(self.UPDATE_INTERVAL, self.publish_goal_positions)

        self.ZERO_POSITION = 2048.0
        self.AMPLITUDE = 100.0  # ±100 pulse range
        self.FREQUENCY = 2 # 1 Hz sine wave

        self.time_elapsed = 0.0  # Time counter

    def publish_goal_positions(self):
        goal_positions = [self.ZERO_POSITION] * 12  # Initialize with neutral positions

        for motor_id in range(12):
            goal_positions[motor_id] = self.ZERO_POSITION + self.AMPLITUDE * math.sin(2 * math.pi * self.FREQUENCY * self.time_elapsed)

        msg = Float32MultiArray()
        msg.data = goal_positions
        self.publisher_.publish(msg)

        self.time_elapsed += self.UPDATE_INTERVAL  # Update time for sine wave progression

        # self.get_logger().info(f'Published sine wave goal positions: {goal_positions}')

def main(args=None):
    rclpy.init(args=args)
    node = SinusoidalGoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

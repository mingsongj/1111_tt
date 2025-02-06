import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class SinusoidalGoalPublisher(Node):
    def __init__(self):
        super().__init__('sinusoidal_goal_publisher')

        self.publisher_ = self.create_publisher(Float32MultiArray, 'goal_positions', 10)
        self.UPDATE_INTERVAL = 1  # 100 Hz for smooth motion
        self.timer = self.create_timer(self.UPDATE_INTERVAL, self.publish_goal_positions)

        self.ZERO_POSITION = 2048.0
        self.PI_ENCODED = 0.0
        self.NEG_PI_ENCODED = 4096.0

        self.motor_positions = {
            0: 1.3,   # fl_a_joint
            1: -1.3,   # fl_h_joint
            2: 1.3,   # rl_a_joint
            3: -1.3,  # rl_h_joint
            4: 0.15,    # rr_a_joint
            5: -0.15,   # rr_h_joint
            6: 0.15,   # fr_a_joint
            7: -0.15  # fr_h_joint
        }

        self.fixed_motors = [8,9,10,11]  # Motors that should always be at 0 position

    def to_encoder_value(self, position):
        return float(self.ZERO_POSITION - (position / math.pi) * (self.ZERO_POSITION - self.PI_ENCODED))

    def publish_goal_positions(self):
        goal_positions = [float(self.ZERO_POSITION)] * 12  # Initialize with neutral positions

        for motor_id, position in self.motor_positions.items():
            goal_positions[motor_id] = self.to_encoder_value(position)

        for fixed_motor in self.fixed_motors:
            goal_positions[fixed_motor] = self.ZERO_POSITION  # Ensure fixed motors remain at 2048

        msg = Float32MultiArray()
        msg.data = [float(pos) for pos in goal_positions]
        self.publisher_.publish(msg)

        # self.get_logger().info(f'Published fixed goal positions: {goal_positions}')

def main(args=None):
    rclpy.init(args=args)
    node = SinusoidalGoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

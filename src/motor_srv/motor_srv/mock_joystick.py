import rclpy
import pygame
from rclpy.node import Node
from geometry_msgs.msg import Twist

def apply_deadzone(value, threshold=0.1):
    """Applies a deadzone to filter out small noise values."""
    return value if abs(value) > threshold else 0.0

def clamp(value, min_value, max_value):
    """Clamps a value within the specified range."""
    return max(min_value, min(value, max_value))

class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('mock_joystick')
        self.publisher_ = self.create_publisher(Twist, '/motor_commands', 10)

        # Initialize pygame for joystick input
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick detected. Please connect a joystick and restart the node.")
            rclpy.shutdown()
            return

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        self.get_logger().info(f"Joystick '{self.joystick.get_name()}' initialized.")

        self.timer = self.create_timer(0.1, self.joystick_loop)

    def get_joystick_input(self):
        """Reads joystick input and maps it to desired control values."""
        pygame.event.pump()  # Process joystick events

        x_axis = self.joystick.get_axis(0)  # Left stick X-axis (strafe)
        y_axis = self.joystick.get_axis(1)  # Left stick Y-axis (forward/backward)
        angular_axis = self.joystick.get_axis(3)  # Right stick X-axis (rotation)

        # Apply deadzone filtering
        A_x = apply_deadzone(-y_axis)  # Forward/backward (invert for proper control)
        A_y = apply_deadzone(x_axis)   # Left/right movement
        ang_vel = apply_deadzone(angular_axis)  # Angular rotation

        # Scale values appropriately
        A_x = clamp(A_x * 0.8, -0.6, 0.6)  # Scale [-1,1] to [-0.6,0.6]
        A_y = clamp(A_y * 0.6, -0.2, 0.2)  # Scale [-1,1] to [-0.2,0.2]
        ang_vel = clamp(ang_vel * 1.0, -0.3, 0.3)  # Scale [-1,1] to [-0.3,0.3]

        return A_x, A_y, ang_vel

    def joystick_loop(self):
        """Reads joystick inputs and publishes control commands."""
        A_x, A_y, ang_vel = self.get_joystick_input()

        # Create and publish Twist message
        twist_msg = Twist()
        twist_msg.linear.x = A_x
        twist_msg.linear.y = A_y
        twist_msg.angular.z = ang_vel

        self.publisher_.publish(twist_msg)

        # self.get_logger().info(f"Joystick Control - X: {A_x:.2f}, Y: {A_y:.2f}, Angular: {ang_vel:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = JoystickControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        pygame.quit()  # Clean up pygame resources
        rclpy.shutdown()

if __name__ == "__main__":
    main()

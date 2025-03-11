import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

def apply_deadzone(value, threshold=0.1):
    """Applies a deadzone to filter out small noise values."""
    return value if abs(value) > threshold else 0.0

def clamp(value, min_value, max_value):
    """Clamps a value within the specified range."""
    return max(min_value, min(value, max_value))

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        self.publisher_ = self.create_publisher(Twist, '/motor_commands', 10)

        self.A_x = 0.0
        self.A_y = 0.0
        self.ang_vel = 0.0
        self.height = 0.2  # New height variable
        self.speed_increment = 0.25
        self.angular_increment = 0.02
        self.height_increment = 0.01  # Smaller increment for fine control

        # Store previous values for change detection
        self.prev_A_x = self.A_x
        self.prev_A_y = self.A_y
        self.prev_ang_vel = self.ang_vel
        self.prev_height = self.height

        self.get_logger().info("Keyboard control node started.")
        self.get_logger().info("W/S: A_x, A/D: A_y, ←/→: ang_vel, ↑/↓: height, Q: Quit")

        # Store terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # Start ROS timer to continuously publish commands (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

    def get_key(self):
        """Reads a single key press without requiring Enter"""
        key = None
        if select.select([sys.stdin], [], [], 0.1)[0]:  # Check if key is pressed
            key = sys.stdin.read(1)  # Read one character
            if key == '\x1b':  # Arrow keys detection (Escape sequence)
                key += sys.stdin.read(2)
        return key

    def control_loop(self):
        """Continuously publishes movement commands and reads key inputs"""
        key = self.get_key()
        updated = False  # Flag to track if a value changed

        # Movement logic based on key press
        if key == 'w':
            self.A_x = clamp(self.A_x + self.speed_increment, -1.0, 1.0)
            updated = True
        elif key == 's':
            self.A_x = clamp(self.A_x - self.speed_increment, -1.0, 1.0)
            updated = True
        elif key == 'a':
            self.A_y = clamp(self.A_y + self.speed_increment, -1.0, 1.0)
            updated = True
        elif key == 'd':
            self.A_y = clamp(self.A_y - self.speed_increment, -1.0, 1.0)
            updated = True
        elif key == '\x1b[D':  # Left arrow
            self.ang_vel = clamp(self.ang_vel + self.angular_increment, -1.0, 1.0)
            updated = True
        elif key == '\x1b[C':  # Right arrow
            self.ang_vel = clamp(self.ang_vel - self.angular_increment, -1.0, 1.0)
            updated = True
        elif key == '\x1b[A':  # Up arrow
            self.height = clamp(self.height + self.height_increment, 0.1, 0.28)
            updated = True
        elif key == '\x1b[B':  # Down arrow
            self.height = clamp(self.height - self.height_increment, 0.1, 0.28)
            updated = True
        elif key == 'x':  # Reset all values to zero
            self.A_x = 0.0
            self.A_y = 0.0
            self.ang_vel = 0.0
            #self.height = 0.0
            updated = True
            self.get_logger().info("All values reset to zero.")

        elif key == 'q':  # Quit the program
            self.get_logger().info("Exiting keyboard control node.")
            rclpy.shutdown()

        # Publish Twist message
        twist_msg = Twist()
        twist_msg.linear.x = apply_deadzone(self.A_x)
        twist_msg.linear.y = apply_deadzone(self.A_y)
        twist_msg.linear.z = apply_deadzone(self.ang_vel)
        twist_msg.angular.z = apply_deadzone(self.height)
        self.publisher_.publish(twist_msg)

        # Only log changes in values
        if updated:
            self.get_logger().info(f"A_x: {self.A_x:.2f}, A_y: {self.A_y:.2f}, ang_vel: {self.ang_vel:.2f}, height: {self.height:.2f}")


        # Store previous values
        self.prev_A_x = self.A_x
        self.prev_A_y = self.A_y
        self.prev_ang_vel = self.ang_vel
        self.prev_height = self.height

    def destroy_node(self):
        """Restore terminal settings on exit"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    
    try:
        rclpy.spin(node)  # Keep node running
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

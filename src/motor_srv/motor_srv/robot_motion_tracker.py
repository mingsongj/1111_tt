import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32
import math
import numpy as np
from collections import deque

class RobotMotionTracker(Node):
    def __init__(self):
        super().__init__('robot_motion_tracker')

        # Subscribe to SLAM odometry topic for position tracking
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/visual_slam/vis/slam_odometry',
            self.odom_callback,
            10)
        
        # Subscribe to velocity topic for real-time speed
        self.velocity_subscription = self.create_subscription(
            TwistStamped,
            '/visual_slam/vis/velocity',
            self.velocity_callback,
            10)
        
        # Publishers
        self.distance_publisher = self.create_publisher(Float32, '/robot/distance_traveled', 10)
        self.speed_publisher = self.create_publisher(Float32, '/robot/speed', 10)

        # Variables for distance tracking
        self.total_distance = 0.0
        self.position_history = deque(maxlen=5)  # Store last 5 valid position readings
        self.previous_position = None  # Last valid position

        # Distance change thresholds
        self.max_distance_change = 0.2  # Maximum allowed movement per frame
        self.min_distance_threshold = 0.005  # Minimum movement to be considered valid
        self.max_position_change_per_frame = 0.5  # Maximum allowed single-axis position jump

    def odom_callback(self, msg):
        """ Callback function for odometry data (position tracking). """
        current_position = np.array([
            msg.pose.pose.position.x, 
            msg.pose.pose.position.y, 
            msg.pose.pose.position.z
        ])

        # Remove data with sudden large changes
        if self.previous_position is not None:
            delta = np.abs(current_position - self.previous_position)
            if np.any(delta > self.max_position_change_per_frame):  # If any axis jumps too much
                self.get_logger().warn(f"Skipping sudden jump: Δx={delta[0]:.3f}, Δy={delta[1]:.3f}, Δz={delta[2]:.3f}")
                return  # Ignore this frame

        # Store the valid position reading
        self.position_history.append(current_position)
        self.previous_position = current_position  # Update last valid position

        # Only start calculating distance after at least 5 readings are stored
        if len(self.position_history) == 5:
            # Compute the average of the last 5 valid positions
            avg_position = np.mean(self.position_history, axis=0)
            
            # Compare with the new position
            distance = np.linalg.norm(current_position - avg_position)

            # Apply movement constraints
            if self.min_distance_threshold <= distance <= self.max_distance_change:
                self.total_distance += distance
                self.get_logger().info(f"Distance moved: {distance:.4f} m, Total: {self.total_distance:.4f} m")
            else:
                #self.get_logger().warn(f"Position change ignored: {distance:.4f} m (out of range)")

            # Publish total distance traveled
            distance_msg = Float32()
            distance_msg.data = self.total_distance
            self.distance_publisher.publish(distance_msg)

    def velocity_callback(self, msg):
        """ Callback function for velocity data (speed tracking). """
        vx, vy, vz = msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z
        speed = math.sqrt(vx**2 + vy**2 + vz**2)

        # Publish speed directly (no smoothing applied)
        speed_msg = Float32()
        speed_msg.data = speed
        self.speed_publisher.publish(speed_msg)

        self.get_logger().info(f"Speed: {speed:.4f} m/s")

def main(args=None):
    rclpy.init(args=args)
    node = RobotMotionTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

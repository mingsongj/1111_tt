import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter
from collections import deque
import threading

class ImuProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')
        plt.ion()  # Turn on interactive mode for real-time updating
        self.fig, self.axs = plt.subplots(3, 1, figsize=(10, 8))

        # Define compatible QoS for sensor topics (best effort, volatile durability)
        sensor_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers for acceleration and gyroscope data with appropriate QoS
        self.gyro_sub = self.create_subscription(
            Imu,
            '/camera/gyro/sample',
            self.gyro_callback,
            sensor_qos
        )

        self.accel_sub = self.create_subscription(
            Imu,
            '/camera/accel/sample',
            self.accel_callback,
            sensor_qos
        )

        # Data storage for acceleration readings
        self.linear_accel = np.zeros(3)
        self.angular_velocity = np.zeros(3)

        # Gravity estimation variable (to store the first measurement)
        self.initial_gravity = None

        # Kalman Filter setup for velocity estimation
        self.kf = KalmanFilter(dim_x=3, dim_z=3)
        self.kf.F = np.eye(3)  # State transition matrix (constant velocity model)
        self.kf.H = np.eye(3)  # Measurement function (direct observation)
        self.kf.P *= 1000  # Initial covariance (high uncertainty)
        self.kf.R = np.eye(3) * 5  # Measurement noise
        self.kf.Q = np.eye(3) * 0.01  # Process noise covariance
        self.kf.x = np.zeros((3, 1))  # Initial velocity state

        # Time tracking for integration using ROS 2 clock
        self.last_time = None
        self.velocity = np.zeros(3)

        # Store data for plotting
        self.time_data = deque(maxlen=100)  # Store latest 100 data points
        self.vel_x_data = deque(maxlen=100)
        self.vel_y_data = deque(maxlen=100)
        self.vel_z_data = deque(maxlen=100)

        self.get_logger().info("IMU Processor Node Initialized with BEST_EFFORT QoS")

        # Timer to process data at 10 Hz
        # self.timer = self.create_timer(0.1, self.process_data)

        # Start plotting in a separate thread
        self.plot_thread = threading.Thread(target=self.plot_velocity)
        self.plot_thread.daemon = True
        self.plot_thread.start()

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

    def gyro_callback(self, msg):
        self.angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

    def process_data(self):
        # Get current time from ROS 2 clock in seconds
        current_time = self.get_clock().now().nanoseconds / 1e9

        if self.last_time is None or self.initial_gravity is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        self.last_time = current_time

        # Kalman filter prediction step
        self.kf.predict()

        # Kalman filter update step (measurement = acceleration * dt)
        accel_measurement = self.linear_accel.reshape((3, 1)) * dt
        self.kf.update(accel_measurement)

        # Extract the estimated velocity
        self.velocity = self.kf.x.flatten()

        # Store data for plotting
        self.time_data.append(current_time)
        self.vel_x_data.append(self.velocity[0])
        self.vel_y_data.append(self.velocity[1])
        self.vel_z_data.append(self.velocity[2])

        # Plot the velocity in real time
        self.axs[0].cla()
        self.axs[1].cla()
        self.axs[2].cla()

        self.axs[0].plot(self.time_data, self.vel_x_data, label='Velocity X', color='r')
        self.axs[1].plot(self.time_data, self.vel_y_data, label='Velocity Y', color='g')
        self.axs[2].plot(self.time_data, self.vel_z_data, label='Velocity Z', color='b')

        self.axs[0].set_title('Velocity X over Time')
        self.axs[1].set_title('Velocity Y over Time')
        self.axs[2].set_title('Velocity Z over Time')

        self.axs[0].set_ylabel('Velocity (m/s)')
        self.axs[1].set_ylabel('Velocity (m/s)')
        self.axs[2].set_ylabel('Velocity (m/s)')
        self.axs[2].set_xlabel('Time (s)')

        self.axs[0].legend()
        self.axs[1].legend()
        self.axs[2].legend()

        plt.draw()
        plt.pause(0.01)  # Allows the plot to update dynamically

        self.get_logger().info(f"Filtered Velocity: {self.velocity}")

    def plot_velocity(self):
        plt.ion()  # Enable interactive mode
        fig, axs = plt.subplots(3, 1, figsize=(10, 8))

        while rclpy.ok():
            if len(self.time_data) > 1:
                axs[0].cla()
                axs[1].cla()
                axs[2].cla()

                axs[0].plot(self.time_data, self.vel_x_data, label='Velocity X', color='r')
                axs[1].plot(self.time_data, self.vel_y_data, label='Velocity Y', color='g')
                axs[2].plot(self.time_data, self.vel_z_data, label='Velocity Z', color='b')

                axs[0].set_title('Velocity X over Time')
                axs[1].set_title('Velocity Y over Time')
                axs[2].set_title('Velocity Z over Time')

                axs[0].set_ylabel('Velocity (m/s)')
                axs[1].set_ylabel('Velocity (m/s)')
                axs[2].set_ylabel('Velocity (m/s)')
                axs[2].set_xlabel('Time (s)')

                axs[0].legend()
                axs[1].legend()
                axs[2].legend()

                plt.pause(0.1)

        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = ImuProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

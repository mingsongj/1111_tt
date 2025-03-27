import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import math
import csv
import os
from datetime import datetime

class COTCalculatorNode(Node):
    def __init__(self):
        super().__init__('cot_calculator_node')

        # QoS profile
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.current_sub = self.create_subscription(
            Float32MultiArray, 'dynamixel_current', self.current_callback, sensor_qos)
        self.lat_sub = self.create_subscription(
            Float32, 'arduino/latitude', self.lat_callback, sensor_qos)
        self.lon_sub = self.create_subscription(
            Float32, 'arduino/longitude', self.lon_callback, sensor_qos)

        # Publisher for COT
        self.cot_pub = self.create_publisher(Float32, 'arduino/cot', sensor_qos)

        # Variables
        self.latest_currents = [0.0] * 12  # mA, assuming 12 motors, initialize all to 0
        self.latest_power = 0.0  # mW, calculated from current * 12V
        self.latest_lat = 0.0    # degrees
        self.latest_lon = 0.0    # degrees
        self.prev_lat = None     # Initialize as None until first valid reading
        self.prev_lon = None
        self.prev_time = None
        self.total_energy = 0.0  # mJ
        self.total_distance = 0.0  # meters
        self.has_valid_gps = False

        # Timer to calculate COT every 1 second
        self.timer = self.create_timer(0.1, self.calculate_and_publish_cot)

        # CSV setup
        self.csv_file = open(f'demo_campus_nochange_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # Updated CSV header to include currents for all 12 motors and their sum
        header = ['time', 'cot_mJ_per_m', 'distance_m', 'power_mW'] + \
                 [f'current_mA_{i}' for i in range(12)] + ['total_current_mA']
        self.csv_writer.writerow(header)
        self.get_logger().info(f"Writing data to CSV: {self.csv_file.name}")

        # Log subscriptions
        self.get_logger().info("Subscribed to /dynamixel_current, /arduino/latitude, /arduino/longitude")
        self.get_logger().info("Publishing to /arduino/cot")

    def current_callback(self, msg):
        """Process current data and calculate total power"""
        if len(msg.data) % 2 == 0:  # Ensure pairs of id, current
            currents = {}
            for i in range(0, len(msg.data), 2):
                motor_id = int(msg.data[i])
                current = msg.data[i + 1]  # in mA
                currents[motor_id] = current
            
            # Update latest_currents for all 12 motors
            for i in range(12):
                self.latest_currents[i] = currents.get(i, 0.0)
            
            # Calculate total power (mW) = sum(current in mA * 12V)
            total_current = sum(abs(c) for c in self.latest_currents)  # Sum absolute currents
            self.latest_power = total_current * 12.0  # mA * V = mW
            self.get_logger().debug(f"Received currents: {self.latest_currents}, Total power: {self.latest_power} mW")
        else:
            self.get_logger().warn("Invalid current data format received")

    def lat_callback(self, msg):
        self.latest_lat = msg.data
        self.get_logger().debug(f"Received latitude: {self.latest_lat}")

    def lon_callback(self, msg):
        self.latest_lon = msg.data
        self.get_logger().debug(f"Received longitude: {self.latest_lon}")

    def calculate_and_publish_cot(self):
        """ Calculate and publish COT, log to CSV with power and currents """
        current_time = self.get_clock().now()
        time_str = current_time.to_msg().sec + current_time.to_msg().nanosec / 1e9  # Unix timestamp in seconds

        # Calculate total current for logging
        total_current = sum(abs(c) for c in self.latest_currents)

        if self.latest_lat == 0.0 and self.latest_lon == 0.0:
            self.cot_pub.publish(Float32(data=0.0))
            #self.get_logger().info(f"COT: 0.00 mJ/m (No GPS fix yet)")
            # Log with all currents and their sum
            self.csv_writer.writerow(
                [time_str, 0.0, self.total_distance, self.latest_power] + 
                self.latest_currents + [total_current]
            )
            self.csv_file.flush()
            self.prev_time = current_time
            return

        if self.prev_lat is None or self.prev_lon is None:
            self.prev_lat = self.latest_lat
            self.prev_lon = self.latest_lon
            self.prev_time = current_time
            self.has_valid_gps = True
            self.cot_pub.publish(Float32(data=0.0))
            #self.get_logger().info(f"COT: 0.00 mJ/m (First valid GPS: Lat={self.latest_lat}, Lon={self.latest_lon})")
            # Log with all currents and their sum
            self.csv_writer.writerow(
                [time_str, 0.0, self.total_distance, self.latest_power] + 
                self.latest_currents + [total_current]
            )
            self.csv_file.flush()
            return

        # Calculate time difference (seconds)
        time_delta = (current_time - self.prev_time).nanoseconds / 1e9

        # Integrate power to get energy (mW * s = mJ)
        energy = self.latest_power * time_delta
        self.total_energy += energy

        # Calculate distance (meters)
        distance = self.haversine(self.prev_lat, self.prev_lon, self.latest_lat, self.latest_lon)

        # Sanity check: reject distances > 1000m in 1s
        if distance > 1000.0:
            self.get_logger().warn(
                f"Rejected unrealistic distance: {distance:.2f} m "
                f"(Lat: {self.prev_lat}->{self.latest_lat}, Lon: {self.prev_lon}->{self.latest_lon})"
            )
            # Log with all currents and their sum
            self.csv_writer.writerow(
                [time_str, 0.0, self.total_distance, self.latest_power] + 
                self.latest_currents + [total_current]
            )
            self.csv_file.flush()
            self.prev_time = current_time
            return

        self.total_distance += distance

        # Calculate and publish COT
        if self.total_distance > 0:
            cot = self.total_energy / self.total_distance
        else:
            cot = 0.0

        self.cot_pub.publish(Float32(data=cot))
        # self.get_logger().info(
        #     f"COT: {cot:.2f} mJ/m, Energy: {self.total_energy:.2f} mJ, Distance: {self.total_distance:.2f} m, "
        #     f"Power: {self.latest_power:.2f} mW, Currents: {self.latest_currents}, Total Current: {total_current:.2f} mA"
        # )

        # Write to CSV with power, all currents, and their sum
        self.csv_writer.writerow(
            [time_str, cot, self.total_distance, self.latest_power] + 
            self.latest_currents + [total_current]
        )
        self.csv_file.flush()

        # Update previous values
        self.prev_lat = self.latest_lat
        self.prev_lon = self.latest_lon
        self.prev_time = current_time

    def haversine(self, lat1, lon1, lat2, lon2):
        """ Calculate distance between two lat/lon points in meters """
        R = 6371000  # Earth's radius in meters
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad

        a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c
        return distance

    def destroy_node(self):
        """ Cleanup: close CSV file """
        self.csv_file.close()
        self.get_logger().info(f"Closed CSV file: {self.csv_file.name}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = COTCalculatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down COT Calculator Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
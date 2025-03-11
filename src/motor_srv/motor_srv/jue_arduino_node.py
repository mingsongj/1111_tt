import rclpy
from rclpy.node import Node
import serial
import threading
from std_msgs.msg import Float32, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('jue_arduino_node')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Serial parameters
        self.serial_port = '/dev/ttyACM0'  # Adjust based on your system (e.g., '/dev/ttyUSB0' or 'COM3' on Windows)
        self.baud_rate = 115200

        # Publishers for all sensor data
        self.height_pub = self.create_publisher(Float32, 'arduino/height', sensor_qos)
        self.pressure_pub = self.create_publisher(Float32, 'arduino/pressure', sensor_qos)
        self.latitude_pub = self.create_publisher(Float32, 'arduino/latitude', sensor_qos)
        self.longitude_pub = self.create_publisher(Float32, 'arduino/longitude', sensor_qos)
        self.command_sub = self.create_subscription(String, 'arduino/command', self.command_callback, sensor_qos)

        # Connect to serial port
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to {self.serial_port} at {self.baud_rate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            return

        # Start serial reading thread
        self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.serial_thread.start()

    def read_serial(self):
        """ Read data from Arduino serial and publish to ROS2 topics """
        while rclpy.ok():
            try:
                line = self.serial_conn.readline().decode('utf-8').strip()
                # Check for initialization messages first
                if "GNSS 初始化成功" in line or "u-blox GNSS 模块启动" in line:
                    self.get_logger().info(f"Arduino init message: {line}")
                    continue

                # Parse sensor data
                if line.startswith("Height:"):
                    # Split into height, pressure, and GPS sections
                    sections = line.split("; ")
                    if len(sections) != 3:
                        self.get_logger().warn(f"Invalid data format: {line}")
                        continue

                    # Parse height and pressure data
                    height_data = sections[0].replace("Height: ", "")
                    pressure_data = sections[1].replace("Pressure: ", "")
                    gps_data = sections[2].replace("GPS: Lat: ", "").split(", Lon: ")

                    if len(gps_data) != 2:
                        self.get_logger().warn(f"Malformed GPS data: {line}")
                        continue

                    try:
                        # Extract and convert height from mm to m
                        height_mm = float(height_data)
                        height = height_mm / 1000.0  # Convert mm to m
                        pressure = float(pressure_data)

                        self.height_pub.publish(Float32(data=height))
                        self.pressure_pub.publish(Float32(data=pressure))

                        # Extract and publish GPS data
                        latitude = float(gps_data[0])
                        longitude = float(gps_data[1])

                        self.latitude_pub.publish(Float32(data=latitude))
                        self.longitude_pub.publish(Float32(data=longitude))

                        self.get_logger().info(
                            f"Published: Height={height} m, Pressure={pressure}, "
                            f"Lat={latitude}, Lon={longitude}"
                        )
                    except ValueError as e:
                        self.get_logger().warn(f"Failed to parse data: {line}, Error: {e}")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial error: {e}")
                break

    def command_callback(self, msg):
        """ Listen to ROS2 topic and send commands to Arduino """
        command = msg.data.strip()
        if command.startswith("i,") or command.startswith("d,"):
            self.serial_conn.write((command + "\n").encode('utf-8'))
            self.get_logger().info(f"Sent command: {command}")
        else:
            self.get_logger().warn(f"Invalid command received: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Arduino Serial Node...")
    finally:
        node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
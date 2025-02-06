import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String 
import serial
from motor_srv.find_device_path import get_device_path

class ArduinoNode(Node):
    def __init__(self):
        super().__init__('arduino_node')
        self.voltage_publisher = self.create_publisher(Float32, 'voltage', 10)
        self.power_publisher = self.create_publisher(Float32, 'power', 10)
        self.cmd_subscription = self.create_subscription(
            String,
            'pneumatic_command',
            self.command_callback,
            10
        )
        # obtains the actual device_path name using the symbolic name ttyUSB_Arduino
        # device_path = get_device_path("arduino")
        device_path = "/dev/ttyUSB1"
        self.ser = serial.Serial(device_path, 115200)  # Replace with your Arduino's serial port
        self.ser.flushInput()
        self.timer = self.create_timer(0.05, self.publish_sensor_data)  # read at 20Hz (0.05 seconds interval) for 10Hz input from arduino 

    def read_sensor_data(self):
        if self.ser.in_waiting:
            response = self.ser.readline().decode('utf-8').strip()
            data = response.split(',')  # Assuming data is formatted as "voltage,power"
            if len(data) == 2:
                voltage_mV, power_mW = map(float, data)
                voltage_V = voltage_mV / 1000.0  # Convert millivolts to volts
                power_W = power_mW / 1000.0  # Convert milliwatts to watts
                return voltage_V, power_W
        return None, None
    
    def publish_sensor_data(self):
        voltage, power = self.read_sensor_data()
        if voltage is not None and power is not None:
            voltage_msg = Float32()
            voltage_msg.data = voltage
            self.voltage_publisher.publish(voltage_msg)
            self.get_logger().info(f'Published voltage: {voltage:.2f} V')

            power_msg = Float32()
            power_msg.data = power
            self.power_publisher.publish(power_msg)
            self.get_logger().info(f'Published power: {power:.2f} W')

    def command_callback(self, msg):
        command = msg.data
        self.send_command_to_arduino(command)
    
    def send_command_to_arduino(self, command):
        try:
            self.ser.write(command.encode('utf-8'))
            self.get_logger().info(f'Sent command to Arduino: {command}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to send command to Arduino: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

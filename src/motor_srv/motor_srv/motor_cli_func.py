
import sys
import yaml  # Import PyYAML for reading YAML files
import os
from example_interfaces.srv import GaitsSlection
import rclpy
from rclpy.node import Node

class Turtle_Gait_Client(Node):

    def __init__(self):
        super().__init__('turtle_gait_client')
        self.cli = self.create_client(GaitsSlection, 'call_gaits')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GaitsSlection.Request()

    def send_request(self, message, time_delay=0.0, cycle=5, p_gain=0, i_gain=0, d_gain=0, amp_a=0, amp_b=0, offset_a=0, offset_b=0):
        self.req.message = message
        self.req.time_delay = float(time_delay)
        self.req.cycle = int(cycle)
        self.req.p_gain = int(p_gain)
        self.req.i_gain = int(i_gain)
        self.req.d_gain = int(d_gain)
        self.req.amp_a = int(amp_a)
        self.req.amp_b = int(amp_b)
        self.req.offset_a = int(offset_a)
        self.req.offset_b = int(offset_b)
        
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()
    minimal_client = Turtle_Gait_Client()

    # Debugging: Print the current working directory
    print("Current working directory:", os.getcwd())

    # Load parameters from the configuration file
    with open('config.yaml', 'r') as file:
        params = yaml.safe_load(file)

    # Load default values from the config.yaml file
    command = params.get('command', 'xxx')
    time_delay = params.get('time_delay', 0.0)
    cycle = params.get('cycle', 5)
    p_gain = params.get('p_gain', 800)  # Default P gain
    i_gain = params.get('i_gain', 40)   # Default I gain
    d_gain = params.get('d_gain', 1)    # Default D gain
    amp_a = params.get('amp_a', 30)
    amp_b = params.get('amp_b', 20)
    offset_a = params.get('offset_a', 20)
    offset_b = params.get('offset_b', -10)

    # Override config.yaml values with command-line arguments if provided
    if len(sys.argv) > 1:
        command = sys.argv[1]
    if len(sys.argv) > 2:
        time_delay = float(sys.argv[2])
    if len(sys.argv) > 3:
        cycle = int(sys.argv[3])
    if len(sys.argv) > 4:
        p_gain = int(sys.argv[4])
    if len(sys.argv) > 5:
        i_gain = int(sys.argv[5])
    if len(sys.argv) > 6:
        d_gain = int(sys.argv[6])
    if len(sys.argv) > 7:
        amp_a = int(sys.argv[7])
    if len(sys.argv) > 8:
        amp_b = int(sys.argv[8])
    if len(sys.argv) > 9:
        offset_a = int(sys.argv[9])
    if len(sys.argv) > 10:
        offset_b = int(sys.argv[10])

    # Send the request with possibly overridden values
    response = minimal_client.send_request(
        command, 
        time_delay, 
        cycle, 
        p_gain, 
        i_gain, 
        d_gain, 
        amp_a, 
        amp_b, 
        offset_a, 
        offset_b
    )

    minimal_client.get_logger().info('Result: %s' % (response.resp_msg))

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from example_interfaces.srv import GaitsSlection
from std_msgs.msg import Bool  # Import the Bool message type
from motor_srv import find_device_path
from motor_srv.turtle_gait_simple import Robot

class TurtleGaits(Node):

    def __init__(self):
        super().__init__('motor_service')
        # Obtain the actual device name using the symbolic name ttyUSB_MotorControl
        # Initialize the Robot with the correct device path
        self.robot = Robot(servo_ids=list(range(0, 12)), DEVICENAME='/dev/ttyUSB0',BAUDRATE = 4000000)
        # Create a publisher for the "robot_status" topic
        self.publisher_ = self.create_publisher(Bool, 'robot_status', 10)
        # Create the service to handle gait requests
        self.srv = self.create_service(GaitsSlection, 'call_gaits', self.gaits_callback)

    def gaits_callback(self, request, response):
        # if self.is_busy:
        #     self.get_logger().info('Ignoring request. Previous request still in progress.')
        #     response.resp_msg = 'busy'
        #     return response

        if request.message is not None:
            self.is_busy = True

            if request.message == 'circle1':
                self.get_logger().info(f'Incoming request: {request.message}, Time_delay: {request.time_delay}')

                # Set the time_delay (indirect velocity)
                self.robot.set_move_delay(request.time_delay)

                # Set the PID parameters
                self.robot.set_P_gain(request.p_gain)
                self.robot.set_I_gain(request.i_gain)
                self.robot.set_D_gain(request.d_gain)

                # Set the amplitude and offset values
                self.robot.set_amp_a(request.amp_a)
                self.robot.set_amp_b(request.amp_b)
                self.robot.set_origin_a(request.offset_a)
                self.robot.set_origin_b(request.offset_b)

                # Set the cycle value
                self.robot.set_cycle(request.cycle)

                # Set the profile velocity and acceleration (you can adjust these if needed)
                self.robot.set_profile_velocity(0)
                self.robot.set_profile_acc(0)

                # Execute the circular motion
                self.robot.circle()
                response.resp_msg = 'circling_mode1'

            elif request.message == 'off':
                self.get_logger().info(f'Incoming request: {request.message}')
                self.robot.rest()
                response.resp_msg = 'torque_disabled'

            elif request.message == 'on':
                self.get_logger().info(f'Incoming request: {request.message}')
                self.robot.torque_on()
                response.resp_msg = 'torque_enabled'   
                
            elif request.message == 'zero_all':
                self.get_logger().info(f'Incoming request: {request.message}')
                # Execute the zero_all function to set all motors to zero position
                self.robot.zero_all()
                response.resp_msg = 'all_motors_zeroed'

            elif request.message == 'read_all':
                self.get_logger().info(f'Incoming request: {request.message}')

                # Execute the zero_all function to set all motors to zero position
                self.robot.read_all()
                response.resp_msg = f'All motor positions read'

            else:
                self.get_logger().warn(f'Unknown gait requested: {request.message}')
                response.resp_msg = 'unknown gait'

            self.is_busy = False
            return response

def main(args=None):
    rclpy.init(args=args)
    gaits_service = TurtleGaits()
    rclpy.spin(gaits_service)
    gaits_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

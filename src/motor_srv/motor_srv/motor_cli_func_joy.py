import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from example_interfaces.srv import GaitsSlection
import time

class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_controller')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            40) # This reads joystick input
        self.gait = 'walk'
        self.timer = self.create_timer(0.02, self.send_request_signal)  # this sends signal (if is_busy is False)

        self.button_states = [False] * 10
        self.axis_states = [0.0]*8
        
        self.is_busy = False
        self.cli = self.create_client(GaitsSlection, 'call_gaits')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        #this publishes to the pneumatics
        self.pneu_publisher = self.create_publisher(String, '/pneumatic_command', 1)


    def publish_command(self, command_data):
        msg = String()
        msg.data = command_data
        self.pneu_publisher.publish(msg)

    async def send_request(self, message, cycle, bias=0.0, backward= False):
        if self.is_busy:
            # self.get_logger().info('Ignoring request. Previous request still in progress.')
            return None

        self.is_busy = True
        req = GaitsSlection.Request()
        req.message = message
        req.cycle = cycle
        req.bias = bias
        req.backward = backward
        future = self.cli.call_async(req)
        future.add_done_callback(self.request_callback)
    
        return future

    def request_callback(self, future):
        try:
            response = future.result()
            self.is_busy = False
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
        else:
            self.get_logger().info(f'Result: {response}')

    def joy_callback(self, msg):
        # Update button states
        for i in range(10):
            self.button_states[i] = msg.buttons[i] == 1
        # Update axis states
        for i in range(8):
            self.axis_states[i] = msg.axes[i]
            
    async def send_request_signal(self):
        if self.axis_states[5] > -0.5 and self.button_states[0]: #back arrow not pressed and button A pressed 
            self.button_states[0] = False #reset button 
            self.gait = 'walk'
            response = await self.send_request('ready_walk', 1)
            if response:
                self.get_logger().info('Ready to walk.')
        
        elif self.axis_states[5] > -0.5 and self.button_states[1]:  #button B
            self.button_states[1] = False #reset button 
            self.gait = 'crawl'
            response = await self.send_request('ready_crawl', 1)
            if response:
                self.get_logger().info('Ready to crawl.')

        elif self.axis_states[5] > -0.5 and self.button_states[2]:  #button X
            self.button_states[2] = False #reset button 
            self.gait = 'paddle'
            response = await self.send_request('ready_swim', 1)
            if response:
                self.get_logger().info('Ready to swim.')

        elif self.axis_states[5] > -0.5 and self.button_states[3]:  #button Y
            self.button_states[3] = False #reset button 
            self.gait = 'flap'
            response = await self.send_request('ready_swim', 1)
            if response:
                self.get_logger().info('Ready to swim.')

        elif self.axis_states[1] > 0.5: #left joystick forward for forward motion
            self.bias = float(self.axis_states[0])
            response = await self.send_request(self.gait, 1, self.bias, backward=False)
            if response:
                self.get_logger().info(f"{self.gait}{' 1-cycle done.'}")
        
        elif self.axis_states[1] < -0.5: #left joystick pushed back for backward motion
            self.bias = float(self.axis_states[0])
            response = await self.send_request(self.gait, 1, self.bias, backward=True)
            if response:
                self.get_logger().info(f"{self.gait}{' 1-cycle done.'}")

        elif self.button_states[5] and self.button_states[7]: #button 6 and 8 pressed (R1&R2):
            self.button_states[5] = False #reset button 
            self.button_states[7] = False #reset button
            self.publish_command('i, 20') #inflate for 5 seconds
            time.sleep(3) #give the command some time to not swamp Arduino 
        
        elif self.button_states[4] and self.button_states[6]: #button 5 and 7 pressed (L1&L2):
            self.button_states[4] = False #reset button 
            self.button_states[6] = False #reset button 
            self.publish_command('d, 20') #deflate for 10 seconds
            time.sleep(3) #give the command some time to not swamp Arduino 

        elif self.button_states[8]: #button 9 pressed (Select):
            self.button_states[8] = False #reset button 
            self.publish_command('u, 10') #unjam for 10 seconds
            time.sleep(3) #give the command some time to not swamp Arduino 
        
        elif self.button_states[9]: #button 10 pressed (Start):
            self.button_states[9] = False #reset button 
            self.publish_command('j, 15') #jam for 10 seconds
            time.sleep(3) #give the command some time to not swamp Arduino 

        elif self.button_states[9]: #button 10 pressed (Start):
            self.button_states[9] = False #reset button 
            self.publish_command('j, 15') #jam for 10 seconds
            time.sleep(3) #give the command some time to not swamp Arduino 
        
        elif self.axis_states[5] < -0.5 and self.button_states[0]: #if back arrow pressed and button A pressed
            self.button_states[0] = False #reset button 
            self.gait = 'crawl_to_walk'
            response = await self.send_request('crawl_to_walk', 1)
            if response:
                self.get_logger().info('Finished Crawl to Walk transition.')

        elif self.axis_states[5] < -0.5 and self.button_states[1]: #if back arrow pressed and button B pressed
            self.button_states[1] = False #reset button 
            self.gait = 'walk_to_crawl'
            response = await self.send_request('walk_to_crawl', 1)
            if response:
                self.get_logger().info('Finished Walk to Crawl transition.')


def main(args=None):
    rclpy.init(args=args)
    joystick_controller = JoystickController()
    try:
        rclpy.spin(joystick_controller)
    except KeyboardInterrupt:
        pass
    finally:
        joystick_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

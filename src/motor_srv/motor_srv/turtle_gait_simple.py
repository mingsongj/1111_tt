from dynamixel_sdk import robot_mod
import time
import numpy as np
import math

class Robot:
    def __init__(self, servo_ids, DEVICENAME, BAUDRATE = 4000000):
        self.servo_ids = servo_ids
        self.DEVICENAME = DEVICENAME
        self.BAUDRATE = BAUDRATE
        self.robot = robot_mod.USB2Dynamixel_Device(servo_ids=servo_ids, DEVICENAME=DEVICENAME, BAUDRATE=BAUDRATE)
        self.profile_velocity = None
        self.profile_acc = None
        self.move_delay = None
        self.P_gain = 800
        self.I_gain = 40
        self.D_gain = 1
        self.amp_a = 30
        self.amp_b = 20
        self.origin_a = 20
        self.origin_b = -10
        self.cycle = 1  # Number of cycles for the elliptical motion


    def set_profile_velocity(self, velocity):
        self.profile_velocity = velocity
        print(f"Profile Velocity set to: {self.profile_velocity}")
        self.robot.set_profile_velocity(self.profile_velocity, self.servo_ids)

    def set_profile_acc(self, acceler):
        self.profile_acc = acceler
        print(f"Profile Acceleration set to: {self.profile_acc}")
        self.robot.set_profile_acc(self.profile_acc, self.servo_ids)

    def set_move_delay(self, move_delay):
        self.move_delay = move_delay

    def enable_torque(self):
        self.robot.enable_torque(self.servo_ids)

    def disable_torque(self):
        self.robot.disable_torque(self.servo_ids)

    def set_P_gain(self, P_gain):
        self.P_gain = P_gain
        self.robot.set_position_P_gain(self.P_gain, self.servo_ids)

    def set_I_gain(self, I_gain):
        self.I_gain = I_gain
        self.robot.set_position_I_gain(self.I_gain, self.servo_ids)

    def set_D_gain(self, D_gain):
        self.D_gain = D_gain
        self.robot.set_position_D_gain(self.D_gain, self.servo_ids)

    def set_amp_a(self, amp_a):
        self.amp_a = amp_a

    def set_amp_b(self, amp_b):
        self.amp_b = amp_b

    def set_origin_a(self, origin_a):
        self.origin_a = origin_a

    def set_origin_b(self, origin_b):
        self.origin_b = origin_b

    def set_cycle(self, cycle):
        self.cycle = cycle

    def degrees_to_encoder(self, deg):
        return int((deg + 180.0) / (360.0 / 0xFFF))
    
    def encoder_to_degrees(self,encoder_value):
        max_encoder_value = 0xFFF  # 4095
        # Scale the encoder value to the corresponding degree value
        degrees = (encoder_value * (360.0 / max_encoder_value)) - 180.0
        return degrees
    
    def circle(self):
        # Enable torque and set PID gains
        self.enable_torque()
        self.robot.set_position_P_gain(self.P_gain, self.servo_ids)
        self.robot.set_position_I_gain(self.I_gain, self.servo_ids)
        self.robot.set_position_D_gain(self.D_gain, self.servo_ids)

        # Elliptical motion setup
        total_phases = int(self.cycle * 200)  # Adjust the number of points based on cycles
        for phase in np.linspace(0, self.cycle * 2 * math.pi, total_phases):
            angle_a = self.origin_a + self.amp_a * math.cos(phase)
            angle_b = self.origin_b + self.amp_b * math.sin(phase)

            servo_locations = [
                self.degrees_to_encoder(angle_a),
                self.degrees_to_encoder(angle_b),
                self.degrees_to_encoder(10),  # Pitch angle is fixed
                self.degrees_to_encoder(angle_a),
                self.degrees_to_encoder(angle_b),
                self.degrees_to_encoder(10),  # Pitch angle is fixed
                self.degrees_to_encoder(angle_a),
                self.degrees_to_encoder(angle_b),
                self.degrees_to_encoder(10),  # Pitch angle is fixed
                self.degrees_to_encoder(angle_a),
                self.degrees_to_encoder(angle_b),
                self.degrees_to_encoder(10),  # Pitch angle is fixed
            ]
            # print(f"Servo locations: {servo_locations}")
            self.robot.sync_write(type="position", dataDict=servo_locations)
            time.sleep(0.05)

    def zero_all(self):
        # Enable torque and set PID gains
        self.enable_torque()
        self.robot.set_position_P_gain(300, self.servo_ids)
        self.robot.set_position_I_gain(20, self.servo_ids)
        self.robot.set_position_D_gain(1, self.servo_ids)

        # Set all 12 positions to zero
        zero_positions = [self.degrees_to_encoder(0) for _ in range(12)]

        # Write the zero positions to all motors
        self.robot.sync_write(type="position", dataDict=zero_positions)
        # time.sleep(0.05)  # Allow some time for the motors to move to 0 position

    # def read_all(self):
    #     motor_pos=self.robot.sync_read(type="position")
    #     motor_deg=[self.encoder_to_degrees(i) for i in motor_pos]
    #     deg=np.array(motor_deg)
    #     pos=np.array(motor_pos)
    #     # print(deg.astype(int))
    #     print(pos.astype(int))
    def read_all(self):
    # Read motor positions
        motor_pos = self.robot.sync_read(type="position")
        motor_vel = self.robot.sync_read(type="velocity")

        # Convert positions from encoder values to degrees
        motor_deg = [self.encoder_to_degrees(i) for i in motor_pos]

        # Convert positions and velocities to numpy arrays for easy processing
        deg = np.array(motor_deg)
        pos = np.array(motor_pos)
        vel = np.array(motor_vel)

        # Append velocity data to position data
        pos_with_vel = np.concatenate((pos, vel))

        # Print positions and velocities
        print("Positions (encoder values):", pos.astype(int))
        print("Velocities:", vel.astype(int))

        # Optional: Print in degree format
        print("Positions (degrees):", deg.astype(int))


    def rest(self):
        self.disable_torque()

    def torque_on(self):
        self.enable_torque()
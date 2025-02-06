from dynamixel_sdk import robot_mod
import time
import numpy as np
from copy import copy
import signal
import sys, os
import math 
# temporarily commented out, since old Jetson does not have the library
# Will need to put back later.  
import matplotlib.pyplot as plt 
import csv

	
class Robot:
    
	COMM_SUCCESS                = 0                             # Communication Success result value
	settings = {'home_encoder': 0x7FF,
	            'max_encoder': 0xFFF,
	            'deg_per_enc': 360.0 / 0xFFF,
	            'neutral_deg':180.0,
	            'flipped': False            
	            }

	leg_motors = {'FL':[0,2,4],  # a mapping between leg names and motors on that leg: front left, back left, front right, back right
	              'BL':[12,14,16], # the values are multiplied by 2 (to be consistent with simulation)
	              'FR':[6,8,10],  # this will be deprecated soon. 
	              'BR':[18,20,22]}
	phase_dict = {0:'BL', 1:'FL', 2:'BR', 3:'FR'} # almost not needed anymore. Will remove soon. 

	# def get_target_pos(up,forward,down):
	#   target_pos={}
	#   target_pos['FL']=[]
	# tar_degs = [0.3, 0.06, 0,15] # final target for: up, forward, down. 
	tar_degs = [40, 5, 20] # final target for: up, forward, down. Tuned up for reality (radian->degrees)
	shift_deg = -abs(tar_degs[1])*2 
	step_num = 5
	crawl_deg = [40,20] #degree difference for down and back
	# standing_start = [0]*12 #Legs straight down
	standing_start = np.array([10,10,0,10,-10,0,-10,-10,0,-10,10,0]) #Legs straight down

	# crawling_start = np.array([80,0,0,80,0,0,-80,0,0,-80,0,0])#with legs out
	crawling_start = np.array([70,10,0,70,10,0,-70,-10,0,-70,-10,0])#with legs out
	
	# swimming start FR , BR , FL ,BL
	swimming_start = np.array([100,10,0,0,-90,0,-100,-10,0,0,90,0])
	
	# this is used to go from crawling to walking  
	crawl_to_walk_transition = np.array([-20,90,140, -20,90,140, 20,-90,-140, 20,-90,-140])
	
	# these are constant to set to record different values to read from motors
	# as well as arrays to hold the reading
	READ_VEL = False
	READ_CURR = False
	READ_POS = False
	READ_VOLT = False
	vel_data = []
	curr_data = []
	pos_data = []
	volt_data = []
	time_data = []
	start_time = time.time()
	###############################################
	#new gait here:
	# offset_array = [0, math.pi, 0, math.pi]
	offset_array = [0, math.pi, math.pi, 0] #diagonal gait
	offset_array2 = [0, 3, 3, 0] 
	offset_direction = [True,False,True,False]


	amp_array= [25,15,25,15,-25,-15,-25,-15] #low amplitude for floor - OG 3/12/2024
	amp_array2= [25,15,25,-15,-25,-15,-25,15] #low amplitude for floor - OG 3/12/2024

	
	origin_array=[15,10,10,-10,-15,-10,-10,10] #new 
	origin_array2=[20,10,20,-10,-20,-10,-20,10]  

	

	# angle_of_attack_array = [45,45,-45,-45]
	angle_of_attack_array = [0,0,0,0]
	# angle_of_attack_array = [90,90,-90,-90]
    
	# for tuning gaits such that it is no longer a circle, but cropped when touching the ground
	touch_down_angles_walk = [math.pi*2/4, math.pi*6/4] #the first is angle when first touching ground, the second is for when lifting
	touch_down_angles_crawl = [math.pi*3/4, math.pi*5/4] #the first is angle when first touching ground, the second is for when lifting

	shift_array = [10,10,-10,-10]
	# crawl arrays:
	offset_array_crawl=[0,0,0,0]
	# OG
	# amp_array_crawl= [20,20,20, 20,-20,-20,-20,-20]
	# 
	# todo: tune amplitude later
	# amp_array_crawl= [30,30,30, 30,-30,-30,-30,-30] #og
	amp_array_crawl= [25,15,25,15,-25,-15,-25,-15] #og
	# amp_array_crawl= [0,0,0,0,-0,-0,-0,-0] #og

	#OG straight up
	# angle_of_attack_array_crawl= [90,90,-90,-90]
	#OG straight up

	#new flat
	angle_of_attack_array_crawl= [0,0,0,0]


	# origin_array_crawl=[70,10, 70,10, -70,-10,-70,-10]
	# origin_array_crawl=[70,0, 70,0, -70,0,-70,0] #OG
	origin_array_crawl=[65,0, 65,0, -65,0,-65,0] 
	

	def __init__(self, servo_ids, DEVICENAME, leg_speed = 30, move_delay=0.016):
		self.servo_ids = servo_ids
		self.DEVICENAME = DEVICENAME
		# starting a class based on definition in robot_mod.py
		self.robot = robot_mod.USB2Dynamixel_Device(servo_ids=servo_ids, DEVICENAME = DEVICENAME)
		self.num_motors = len(servo_ids)
		self.leg_speed = leg_speed 
		self.move_delay= move_delay


# robot = robot_mod.USB2Dynamixel_Device(servo_ids=list(range(0, 12)), DEVICENAME = '/dev/ttyUSB1')
# num_motors = len(robot.servo_ids)





	def signal_handler(self,sig,frame):
		print('\nExiting..... Saving requested info to file.')
		# cleanup code
		self.save_all()
		sys.exit(0)

	def move_from_traj(self,input_data, cycles, delay):
		# checks the input_data data type 
		if isinstance(input_data, np.ndarray):
			if (input_data.shape[1]==3):
				swim_data=input_data
			else:
				raise ValueError("Input array is not the correct size. It should have the shape (numer_of_instructions,3).")
		elif isinstance(input_data, str):
			try:
				# this assumes the first column contains timestamps so the next three columns are used
				# this also skips the first/header row 
				swim_data = np.genfromtxt(input_data,delimiter=',', skip_header=1, usecols=(1,2,3))
				swim_data= swim_data.astype(int)
			except (OSError, ValueError) as e:
				pass
				# raise IOError(f"Error reading CSV file: {str(e)}")
		else:
			raise ValueError("Invalid input type. Enter a NumPy array or a CSV filepath")
		mid_point= swim_data.shape[0] // 2
		# swim_deg= self.encoder_to_degrees(swim_data)
		# mirrored_data=self.degrees_to_encoder(-swim_deg)

		swim_deg= swim_data
		mirrored_data=-swim_deg



		# creates an array to hold the encoder positions for all 12 motors
		motor_pos_shape = (swim_data.shape[0], 12)
		motor_pos= np.zeros(motor_pos_shape)
		# populates instruction array
		motor_pos[:, :3]= swim_data

		# makes it so that the diagonal legs are 180 degrees out of phase
		# two legs read the encoder instructions normallly
		# while the other two legs start reading halfway through the instruction set 
		# and then loop back to the beginning 
		# if there is an even number of rows
		if motor_pos.shape[0]%2 == 0:
			motor_pos[:mid_point, 3:6] = swim_data[mid_point:, :]
			motor_pos[mid_point:, 3:6] = swim_data[:mid_point, :]
			
			motor_pos[:mid_point, 6:9] = mirrored_data[mid_point:, :]
			motor_pos[mid_point:, 6:9] = mirrored_data[:mid_point, :]
		# if there is an odd number of rows 
		else:
			motor_pos[:mid_point, 3:6] = swim_data[mid_point+1:, :]
			motor_pos[mid_point+1:, 3:6] = swim_data[:mid_point, :]
			
			motor_pos[:mid_point, 6:9] = mirrored_data[mid_point+1:, :]
			motor_pos[mid_point+1:, 6:9] = mirrored_data[:mid_point, :]
		
		motor_pos[:, 9:]=mirrored_data

		motor_pos=motor_pos.astype(int)
		print(motor_pos_shape)
		print(motor_pos)
		# self.zero_motors()
		cycle = 0
		while cycle <= cycles:
			self.move(motor_pos, mode="degrees", velocity=30, delay = delay)
			cycle = cycle + 1 

	def new_walk(self, cycles):
		self.robot.enable_torque(self.robot.servo_ids)
		# self.robot.set_profile_velocity(50, self.robot.servo_ids) #set walking (leg move) max speed
		self.robot.set_position_P_gain(400, self.robot.servo_ids)
		self.robot.set_profile_velocity(60, self.robot.servo_ids)

		# amp_A= -Robot.amp_array[0]
		# amp_B = -Robot.amp_array[1]
		amp_A = 25
		amp_B = 15
		# origin_A = -Robot.origin_array[0]
		origin_A = 5
		origin_B = -15 #not sure if non-zero values work
		touch_down_angles = Robot.touch_down_angles_walk
		skip_top = 5
		skip_bottom = 2
		delay = 0.1
		angle_of_attack = Robot.angle_of_attack_array[0]
		array = self.leg_array(amp_A, amp_B, origin_A, origin_B, touch_down_angles, skip_top, skip_bottom, angle_of_attack)
		self.move_from_traj(array,cycles, delay)
		

	# 	return np_data
	
	def leg_array(self, amp_A, amp_B, origin_A, origin_B, touch_down_angles, skip_top, skip_bottom, angle_of_attack):
		granularity = 200  # Adjust the granularity as needed
		phases = [2 * math.pi * i / granularity for i in range(granularity + 1)]

		data = []

		# for i, phase in enumerate(phases):
		# 	angle_A, angle_B = self.leg_circle(phase, amp_A, amp_B, origin_A, origin_B, touch_down_angles)
		# 	data.append([phase, angle_A, angle_B, angle_of_attack])

		for i, phase in enumerate(phases):
			if touch_down_angles[0] < phase < touch_down_angles[1]:
				if i % (skip_bottom+1) == 0:
					angle_A, angle_B = self.leg_circle(phase, amp_A, amp_B, origin_A, origin_B, touch_down_angles)
					data.append([phase, angle_A, angle_B, angle_of_attack])
			else:
				if i % (skip_top+1) == 0:
					angle_A, angle_B = self.leg_circle(phase, amp_A, amp_B, origin_A, origin_B, touch_down_angles)
					data.append([phase, angle_A, angle_B, angle_of_attack])

		np_data = np.array(data, dtype=np.float64)

		# Calculate the average Euclidean distance for points within touch_down_angles phases
		avg_distance_within_phases = self.calculate_average_distance(np_data, touch_down_angles)

		# Calculate the average Euclidean distance for points outside touch_down_angles phases
		avg_distance_outside_phases = self.calculate_average_distance(np_data, touch_down_angles, within=False)

		# Filter the data based on phase and the average distance
		filtered_data = [np_data[0]]  # Initialize with the first point

		for i in range(1, len(np_data)):
			prev_point = filtered_data[-1]
			current_point = np_data[i]
			euclidean_distance = np.linalg.norm(current_point[1:3] - prev_point[1:3])

			# Determine the average distance based on the phase
			if current_point[0] < touch_down_angles[0] or current_point[0] > touch_down_angles[1]:
				average_distance = avg_distance_outside_phases
			else:
				average_distance = avg_distance_within_phases

			# Only add the point to filtered_data if the distance is greater than the average
			if euclidean_distance > average_distance * 0.75: # 0.75 is tolerance 
				filtered_data.append(current_point)

		filtered_data = np.array(filtered_data, dtype=np.float64)
		filtered_data = filtered_data[:, 1:] #remove phase column

		np.savetxt('angles.csv', filtered_data, delimiter=',', header="angle_A,angle_B,angle_of_attack", comments='')

		# Plot and save the data
		plt.figure(figsize=(8, 6))
		plt.scatter(filtered_data[:, 0], filtered_data[:, 1], marker='o')
		plt.xlabel('A')
		plt.ylabel('B')
		plt.legend()
		plt.grid(True)
		plt.savefig('plot.jpg')
		plt.close()

		return filtered_data

	def calculate_average_distance(self, data, touch_down_angles, within=True):
		distances = []
		for i in range(1, len(data)):
			prev_point = data[i - 1]
			current_point = data[i]
			euclidean_distance = np.linalg.norm(current_point[1:3] - prev_point[1:3])

			if (within and touch_down_angles[0] < current_point[0] < touch_down_angles[1]) or (not within and (current_point[0] < touch_down_angles[0] or current_point[0] > touch_down_angles[1])):
				distances.append(euclidean_distance)

		if len(distances) == 0:
			return 0
		return sum(distances) / len(distances)
	
	def leg_circle(self, phase, amp_A, amp_B, origin_A, origin_B, touch_down_angles):
		# granularity=4
		phase = phase % (2 * math.pi)
		angle_A = origin_A + amp_A * math.cos(phase)
		angle_B = origin_B + amp_B * math.sin(phase)
		
		# Round angle_A and angle_B to the nearest granularity (0.2 in this case)
		# angle_A = round(angle_A / granularity) * granularity
		# angle_B = round(angle_B / granularity) * granularity

		if touch_down_angles[0] < phase < touch_down_angles[1]:
			angle_A = origin_A + amp_A * math.cos(touch_down_angles[0])

		return (angle_A, angle_B)
	# some notes: for angle_A , lift, phase 0 (cos =1) is all the way up, and phase pi (cos=-1) is all the way down. 
	# for angle_B, swing, phase 0 is in the middle (sin = 0), phase pi/2 is in the front (sin = 1), 

	def leg_circle2(self, phase, amp_A, amp_B, origin_A, origin_B, shift, direction):
        # Define waypoints as pairs of (x, y)
		waypoints = [
            (origin_A, origin_B + amp_B),
            (origin_A, origin_B + amp_B / 3),
            (origin_A - shift, origin_B + amp_B / 3),
            (origin_A - shift, origin_B - amp_B / 3),
            (origin_A - shift, origin_B - amp_B),
            (origin_A, origin_B - amp_B)
        ]
        
        # Normalize the phase within the 6 intervals
		phase = phase % 6
		if not direction: #reverse the phase if on opposite legs , according to gait
			phase = 6 - phase
        
		if phase < 1:
			circle_angle = phase * math.pi - math.pi / 2 
			angle_A = origin_A + amp_A * math.cos(circle_angle)
			angle_B = origin_B + amp_B * math.sin(circle_angle)
		elif 1 <= phase < 2:
			phase_in_phase = phase - 1
			angle_A, angle_B = self.map_point(phase_in_phase, waypoints[0], waypoints[1])
		elif 2 <= phase < 3:
			phase_in_phase = phase - 2
			angle_A, angle_B = self.map_point(phase_in_phase, waypoints[1], waypoints[2])
		elif 3 <= phase < 4:
			phase_in_phase = phase - 3
			angle_A, angle_B = self.map_point(phase_in_phase, waypoints[2], waypoints[3])
		elif 4 <= phase < 5:
			phase_in_phase = phase - 4
			angle_A, angle_B = self.map_point(phase_in_phase, waypoints[3], waypoints[4])
		else:
			phase_in_phase = phase - 5
			angle_A, angle_B = self.map_point(phase_in_phase, waypoints[4], waypoints[5])

		return angle_A, angle_B

	def map_point(self, t, point1, point2):
		x1, y1 = point1
		x2, y2 = point2
		x = x1 + t * (x2 - x1)
		y = y1 + t * (y2 - y1)
		return x, y



	
	def scale_factor_from_bias(self, bias):	
		if bias>0.0:
			scale_left = (1-bias*0.9)
			scale_right = 1
		elif bias<0.0:
			scale_left = 1
			scale_right = (1+bias*0.9) #because negative
		else: #bias is zero
			scale_left = 1
			scale_right = 1
		return (scale_left, scale_right)

	def from_phase_to_all_legs2(self,phase, offset_array, amp_array, origin_array, angle_of_attack_array, shifts_array, direction_array, bias):
		#amp_array is [amp_A_leg1, amp_B_leg1, amp_A_leg2, etc, amp_B_leg4]
		#origin_array is [origin_A_leg1, origin_B_leg1, etc, origin_B_leg4]
		#phase offset array is an array similar to [0, math.pi/2, 0, math.pi/2]
		#leg 1: front right, motor index: 0 1 2
        
		#leg 1: front right, motor index: 1 2 3 
		FR_lift, FR_forward = self.leg_circle2(phase+self.offset_array2[0],amp_array[0],amp_array[1],origin_array[0],origin_array[1],shifts_array[0],direction_array[0])
		# FR_lift = 
		# FR_forward =
		#leg 2: back right, motor index: 3 4 5
		BR_lift, BR_forward = self.leg_circle2(phase+self.offset_array2[1],amp_array[2],amp_array[3],origin_array[2],origin_array[3],shifts_array[1],direction_array[1])
		# FR_lift = 
		#leg 3: front left, motor index: 6 7 8
		FL_lift, FL_forward = self.leg_circle2(phase+self.offset_array2[2],amp_array[4],amp_array[5],origin_array[4],origin_array[5],shifts_array[2],direction_array[2])
		#leg 4: back left, motor index: 9 10 11
		BL_lift, BL_forward = self.leg_circle2(phase+self.offset_array2[3],amp_array[6],amp_array[7],origin_array[6],origin_array[7],shifts_array[3],direction_array[3])


		FR_angle = angle_of_attack_array[0]
		BR_angle = angle_of_attack_array[1]
		FL_angle = angle_of_attack_array[2]
		BL_angle = angle_of_attack_array[3]

		#bias
		scale_left, scale_right = self.scale_factor_from_bias(bias)

		leg_move_array = [FR_lift, FR_forward*scale_right, FR_angle, BR_lift, BR_forward*scale_right, BR_angle, FL_lift, FL_forward*scale_left, FL_angle,BL_lift, BL_forward*scale_left, BL_angle]
		return leg_move_array
	

	def from_phase_to_all_legs(self,phase, offset_array, amp_array, origin_array, angle_of_attack_array, touch_down_angles, bias):
		#amp_array is [amp_A_leg1, amp_B_leg1, amp_A_leg2, etc, amp_B_leg4]
		#origin_array is [origin_A_leg1, origin_B_leg1, etc, origin_B_leg4]
		#phase offset array is an array similar to [0, math.pi/2, 0, math.pi/2]
		#leg 1: front right, motor index: 0 1 2
        
		#leg 1: front right, motor index: 1 2 3 
		FR_lift, FR_forward = self.leg_circle(phase+offset_array[0],amp_array[0],amp_array[1],origin_array[0],origin_array[1],touch_down_angles)
		# FR_lift = 
		# FR_forward =
		#leg 2: back right, motor index: 3 4 5
		BR_lift, BR_forward = self.leg_circle(phase+offset_array[1],amp_array[2],amp_array[3],origin_array[2],origin_array[3],touch_down_angles)
		# FR_lift = 
		#leg 3: front left, motor index: 6 7 8
		FL_lift, FL_forward = self.leg_circle(phase+offset_array[2],amp_array[4],amp_array[5],origin_array[4],origin_array[5],touch_down_angles)
		#leg 4: back left, motor index: 9 10 11
		BL_lift, BL_forward = self.leg_circle(phase+offset_array[3],amp_array[6],amp_array[7],origin_array[6],origin_array[7],touch_down_angles)


		FR_angle = angle_of_attack_array[0]
		BR_angle = angle_of_attack_array[1]
		FL_angle = angle_of_attack_array[2]
		BL_angle = angle_of_attack_array[3]

		#bias
		scale_left, scale_right = self.scale_factor_from_bias(bias)

		leg_move_array = [FR_lift, FR_forward*scale_right, FR_angle, BR_lift, BR_forward*scale_right, BR_angle, FL_lift, FL_forward*scale_left, FL_angle,BL_lift, BL_forward*scale_left, BL_angle]
		
		return leg_move_array
	
	def test(self,cycles):
		self.robot.enable_torque(self.robot.servo_ids)
		print("enabled torque")
		self.robot.set_profile_velocity(30, self.robot.servo_ids) #set walking (leg move) speed
		print("set profile velocity")

		for i in range(cycles):
			self.robot.sync_write(type="position",dataDict = [self.degrees_to_encoder(20)])
			time.sleep(1)
			self.robot.sync_write(type="position",dataDict = [self.degrees_to_encoder(0)])
			time.sleep(1)

	# Add this function to save data to a CSV file with a timestamp and a custom filename
	def save_data_to_csv(self):
		# Construct the CSV filename using the start time
		start_time = time.time()
		timestamp = time.strftime("%m-%d_%H-%M-%S", time.gmtime(start_time))	
		# Specify the path to the "data" folder parallel to the code's parent folder
		folder_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")

		# Create the "data" folder if it doesn't exist
		if not os.path.exists(folder_path):
			os.makedirs(folder_path)	
		csv_filename = os.path.join(folder_path, f"{timestamp}_data.csv")

		with open(csv_filename, 'a', newline='') as csvfile:
			writer = csv.writer(csvfile)

			if Robot.READ_CURR:
				writer.writerow(["Timestamp", "Current"])
				for timestamp, current in zip(self.time_data, self.curr_data):
					writer.writerow([timestamp, current])

			if Robot.READ_POS:
				writer.writerow(["Timestamp", "Position"])
				for timestamp, position in zip(self.time_data, self.pos_data):
					writer.writerow([timestamp, position])

			if Robot.READ_VEL:
				writer.writerow(["Timestamp", "Velocity"])
				for timestamp, velocity in zip(self.time_data, self.vel_data):
					writer.writerow([timestamp, velocity])

			if Robot.READ_VOLT:
				writer.writerow(["Timestamp", "Voltage"])
				for timestamp, voltage in zip(self.time_data, self.volt_data):
					writer.writerow([timestamp, voltage])

	def ready_walk(self):
		#first enable torque and set speeds
		self.robot.enable_torque(self.robot.servo_ids)
		self.robot.set_profile_velocity(50, self.robot.servo_ids) #set walking (leg move) max speed
		self.robot.set_position_P_gain(800, self.robot.servo_ids)
		self.robot.set_position_I_gain(40, self.robot.servo_ids) #og was 40 
		self.robot.set_position_D_gain(1, self.robot.servo_ids)
		#then go to starting position 
		servo_locations = [self.degrees_to_encoder(x) for x in Robot.standing_start]
		self.robot.sync_write(type = "position", dataDict = servo_locations)

	def ready_crawl(self):
		#first enable torque and set speeds
		self.robot.enable_torque(self.robot.servo_ids)
		self.robot.set_profile_velocity(50, self.robot.servo_ids) #set crawling (leg move) max speed
		self.robot.set_position_P_gain(800, self.robot.servo_ids)
		self.robot.set_position_I_gain(40, self.robot.servo_ids)
		self.robot.set_position_D_gain(1, self.robot.servo_ids)
		#then go to starting position 
		servo_locations = [self.degrees_to_encoder(x) for x in Robot.crawling_start]
		self.robot.sync_write(type = "position", dataDict = servo_locations)

	def ready_swim(self):
		#first enable torque and set speeds
		self.robot.enable_torque(self.robot.servo_ids)
		self.robot.set_profile_velocity(50, self.robot.servo_ids) #set walking (leg move) max speed
		self.robot.set_position_P_gain(800, self.robot.servo_ids)
		self.robot.set_position_I_gain(40, self.robot.servo_ids)
		self.robot.set_position_D_gain(1, self.robot.servo_ids)
		#then go to starting position 
		servo_locations = [self.degrees_to_encoder(x) for x in Robot.swimming_start]
		self.robot.sync_write(type = "position", dataDict = servo_locations)

	def walk(self,cycles,bias=0,backward=False):
		total_phase = cycles*2*math.pi
		# the increments are for position control destinations
		start = 0.0
		increment = 0.1 #OG 

		start_time = time.time()

		if backward:
			phase = total_phase
			target_phase = start
		else:
			phase = start
			target_phase = total_phase

		while abs(phase-target_phase) > increment:
			leg_move_array = self.from_phase_to_all_legs(phase,Robot.offset_array,Robot.amp_array,Robot.origin_array, Robot.angle_of_attack_array, Robot.touch_down_angles_walk, bias)

			pos_enc = [self.degrees_to_encoder(i) for i in leg_move_array]
			self.robot.sync_write(type="position", dataDict = pos_enc)
			
			time.sleep(0.05) #OG
			# time.sleep(0.01) #shortened because recording takes time too 
			if backward:
				phase -= increment
			else:
				phase += increment

			if Robot.READ_CURR == True:
				pres_curr = self.robot.sync_read(type="current")
				Robot.curr_data.append(pres_curr)
			if Robot.READ_POS == True:
				pres_pos = self.robot.sync_read(type="position")
				Robot.pos_data.append(pres_pos)
			if Robot.READ_CURR == True:
				pres_vel = self.robot.sync_read(type="velocity")
				Robot.vel_data.append(pres_vel)
			if Robot.READ_VOLT == True:
				pres_volt = self.robot.sync_read(type="voltage")
				Robot.volt_data.append(pres_volt)
			# if any data recording is on, also record time.
			if any([Robot.READ_CURR, Robot.READ_POS, Robot.READ_VEL, Robot.READ_VOLT]):
				current_time = time.time()
				Robot.time_data.append(current_time - start_time)
			
		if any([Robot.READ_CURR, Robot.READ_POS, Robot.READ_VEL, Robot.READ_VOLT]):
			self.save_data_to_csv()

	def walk2(self,cycles,bias=0,backward=False):
		total_phase = cycles*6
		# the increments are for position control destinations
		start = 0.0
		# increment = 0.1 #OG 
		increment = 0.05 #OG 

		start_time = time.time()

		if backward:
			phase = total_phase
			target_phase = start
		else:
			phase = start
			target_phase = total_phase

		while abs(phase-target_phase) > increment:
			leg_move_array = self.from_phase_to_all_legs2(phase,Robot.offset_array,Robot.amp_array2,Robot.origin_array2, Robot.angle_of_attack_array, Robot.shift_array, Robot.offset_direction, bias)

			pos_enc = [self.degrees_to_encoder(i) for i in leg_move_array]
			self.robot.sync_write(type="position", dataDict = pos_enc)
			
			time.sleep(0.05) #OG
			# time.sleep(0.01) #shortened because recording takes time too 
			if backward:
				phase -= increment
			else:
				phase += increment

			if Robot.READ_CURR == True:
				pres_curr = self.robot.sync_read(type="current")
				Robot.curr_data.append(pres_curr)
			if Robot.READ_POS == True:
				pres_pos = self.robot.sync_read(type="position")
				Robot.pos_data.append(pres_pos)
			if Robot.READ_CURR == True:
				pres_vel = self.robot.sync_read(type="velocity")
				Robot.vel_data.append(pres_vel)
			if Robot.READ_VOLT == True:
				pres_volt = self.robot.sync_read(type="voltage")
				Robot.volt_data.append(pres_volt)
			# if any data recording is on, also record time.
			if any([Robot.READ_CURR, Robot.READ_POS, Robot.READ_VEL, Robot.READ_VOLT]):
				current_time = time.time()
				Robot.time_data.append(current_time - start_time)
			
		if any([Robot.READ_CURR, Robot.READ_POS, Robot.READ_VEL, Robot.READ_VOLT]):
			self.save_data_to_csv()


    # new crawling gait, uses the same procedure as the walking gait     
	def crawl(self, cycles,bias=0, backward = False):
		# self.robot.enable_torque(self.robot.servo_ids)
		# self.robot.set_profile_velocity(20, self.robot.servo_ids)
		# self.robot.set_profile_velocity(20, self.robot.servo_ids) #og

		# self.robot.set_profile_velocity(50, self.robot.servo_ids) #set walking (leg move) max speed
		# self.robot.set_position_P_gain(800, self.robot.servo_ids)
		# self.robot.set_position_I_gain(40, self.robot.servo_ids)
		# self.robot.set_position_D_gain(1, self.robot.servo_ids)

		total_phase = cycles * 2*math.pi 
		# total_increments = cycles*30
		start = 0.0
		increment = 0.1
		# increment = 0.03

		if backward:
			phase = total_phase
			target_phase = start
		else:
			phase = start
			target_phase = total_phase

		while abs(phase-target_phase) > increment:

			leg_move_array = self.from_phase_to_all_legs(phase,Robot.offset_array_crawl,Robot.amp_array_crawl,Robot.origin_array_crawl, Robot.angle_of_attack_array_crawl, Robot.touch_down_angles_crawl, bias)

		# ts_old = time.time()
		# for pos in motor_commands:
			# print(pos)
			pos_enc = [self.degrees_to_encoder(i) for i in leg_move_array]
			self.robot.sync_write(type="position", dataDict = pos_enc)
			time.sleep(0.05)
			if backward:
				phase -= increment
			else:
				phase += increment
    
	# records the encoder values for one limb and exports the data as a .csv file
	# stops recording when the limb stops moving 
	# or records for a specified duration 
	def teach(self, absolute_tolerance = 1, check_delay = 0.5, duration =10, export=False, smooth = False, file_path = "motor_positions.csv", mode= "movement"):
		self.robot.disable_torque(self.robot.servo_ids)
		old_servo_ids= self.robot.servo_ids
		self.robot.servo_ids= list(range(0,3))
		# flags and variables used to check if the limb is still moving
		set_position=True
		tolerance= absolute_tolerance
		last_time_equal= 0
		# delay between checking if the past position is the same as the current position
		equal_delay= check_delay 
		last_pos = np.array([0, 0, 0])
		# keeps track of motor instructions and time
		motor_positions= []
		timestamps= []
		# flags used to mark the first iteration of the while loop
		first_iteration= True
		first_check= False
		if mode == "movement":
			# this while loop will continue running and recording the motor encoder positions until set_position is False
			# this loop will compare the last limb position with the current limb position
			# if these are equal the loop will check again after a given delay
			# if these are equal again, the function stops recording and the data is exported   
			while set_position:
				# in while loop flag 
				if first_iteration:
					print('in while loop')
					first_iteration= False
					initial_time = time.time()
					print('Start recording!')
					time.sleep(1)
				# reads current pos
				pos_data = self.robot.sync_read(type="position")
				# print(pos_data)
				# checks if current reading is the same as past reading
				if (np.allclose(pos_data, last_pos, atol=0, rtol=0)):
					# only updates last_time_equal if it was not updated in last loop iteration
					# otherwise last_time_equal would just keep getting updated and the delay between checks condition would never be satisfied
					if first_check==False:
						last_time_equal=time.time()
						print('last time equal updated')
						print(last_time_equal)
						first_check=True
				elapsed_time= time.time()
				# print(elapsed_time)
				# if motor positions change and the specified delay is satisfied
				# makes it so first check has to happen again
				if not np.allclose(pos_data,last_pos, atol=tolerance, rtol=0) and (elapsed_time - last_time_equal) > equal_delay:
					first_check=False

				# if the first check was completed and the delay is satisfied
				# checks if the last position of the motors is the same as the current position
				# exits out of the while loop if this is true
				if ((elapsed_time - last_time_equal) > equal_delay and first_check): 
					if (np.allclose(last_pos, pos_data, atol=tolerance, rtol=0)):
						set_position=False
				last_pos = pos_data
				motor_positions.append(pos_data)
				timestamps.append(elapsed_time - initial_time)
		elif mode == "time":
			initial_time = time.time()
			end_time= initial_time + duration
			while time.time()<end_time:
				elapsed_time= time.time()
				# reads current pos
				pos_data = self.robot.sync_read(type="position")
				motor_positions.append(pos_data)
				timestamps.append(elapsed_time - initial_time)
		print('Stopped recording.')
		motor_positions_array= np.array(motor_positions)
		data_with_time = np.column_stack((timestamps, motor_positions_array))
		if smooth:
			data_with_time = self.smooth_data(data_with_time)
		if export:
			# defaults to saving in working directory without further specification
			csv_file_path = file_path
			np.savetxt(csv_file_path, data_with_time, delimiter=",", fmt=["%f"] + ["%d"] * motor_positions_array.shape[1])
			print("Successfully saved CSV file", flush = True)
		# returns an array containing the three motor instructions without the timestamps
		self.robot.servo_ids=old_servo_ids
		return motor_positions_array
	
	def smooth_data(data, window_size=5):
		"""
		Smooths the input time series data using a simple moving average.

		Parameters:
		- data: Input time series data (nx4 array or DataFrame).
		- window_size: Size of the moving average window.

		Returns:
		- Smoothed time series data.
		"""
		smoothed_data = np.zeros_like(data)

		for i in range(data.shape[1]):
			smoothed_data[:, i] = np.convolve(data[:, i], np.ones(window_size)/window_size, mode='same')

		return smoothed_data


    # takes in a NumPy array or a CSV filepath containing motor encoder values for the limb with motor ids 1,2, and 3
	# uses these instructions to mirror the motion for the other front flipper
	# the back two flippers retain the same position throughout the swimming gait
	# 	def two_flipper_swim(self,input_data, speed = 50, back_leg_pos=[2500, 1060, 2480], cycles= 5):
	def two_flipper_swim(self,input_data, speed = 50, back_leg_pos=[2048, 1024, 2048], cycles= 5, bias = 0):
		# self.robot.enable_torque(self.servo_ids)
		# self.robot.set_profile_velocity(speed, self.robot.servo_ids) #set walking (leg move) max speed
		# self.robot.set_position_P_gain(800, self.robot.servo_ids)
		# self.robot.set_position_I_gain(40, self.robot.servo_ids)
		# self.robot.set_position_D_gain(1, self.robot.servo_ids)
		# checks the input_data data type 2500, 1067, 2486
		if isinstance(input_data, np.ndarray):
			if (input_data.shape[1]==3):
				swim_data=input_data
			else:
				raise ValueError("Input array is not the correct size. It should have the shape (numer_of_instructions,3).")
		elif isinstance(input_data, str):
			try:
				# this assumes the first column contains timestamps so the next three columns are used
				# this also skips the first/header row 
				swim_data = np.genfromtxt(input_data,delimiter=',', skip_header=1, usecols=(1,2,3))
				swim_data= swim_data.astype(int)
			except (OSError, ValueError) as e:
				pass
				# raise IOError(f"Error reading CSV file: {str(e)}")
		else:
			raise ValueError("Invalid input type. Enter a NumPy array or a CSV filepath")

		# swim_data contains the encoder values for motors 1,2,3 
		# mirrored_front for motors 7,8,9
		# back_data for motors 4,5,6 
		# mirrored_back for motors 10,11,12
		# these names might be a little confusing, but I didn't want to assign left or right to them
		# since the turtle I've been using for testing flips the left and right (compared to how Billy assigned them in other parts of the code)
		# for the turle I've been tesing on:
		# 1,2,3 front left
		# 4,5,6 back left
		# 7,8,9 front right
		# 10,11,12 back right 
		swim_deg= self.encoder_to_degrees(swim_data)
		mirrored_front=self.degrees_to_encoder(-swim_deg)
		back_pos = np.array(back_leg_pos)

		# keeps array shapes consistent
		back_data = np.tile(back_pos, (swim_data.shape[0], 1))
		back_deg= self.encoder_to_degrees(back_data)
		mirrored_back= self.degrees_to_encoder(-back_deg)
		# creates an array to hold the encoder positions for all 12 motors
		motor_pos_shape = (swim_data.shape[0], 12)
		motor_pos= np.zeros(motor_pos_shape)
		# populates instruction array 
		motor_pos[:, :3]=swim_data
		motor_pos[:, 3:6]=back_data
		motor_pos[:, 6:9]=mirrored_front
		motor_pos[:, 9:]=mirrored_back
		motor_pos=motor_pos.astype(int)
		# self.zero_motors()
		cycle=1
		while cycle <= cycles:
			self.move(motor_pos, mode="encoder")
			cycle= cycle+1

	# takes in a NumPy array or a CSV filepath containing motor encoder values for the limb with motor ids 1,2, and 3
	# uses these instructions to create a four flipper swimming gain in which diagonal flippers are in phase 
	# while the left are right sides are 180 degrees out of phase
	def four_flipper_swim(self,input_data, cycles = 5):
		# checks the input_data data type 
		if isinstance(input_data, np.ndarray):
			if (input_data.shape[1]==3):
				swim_data=input_data
			else:
				raise ValueError("Input array is not the correct size. It should have the shape (numer_of_instructions,3).")
		elif isinstance(input_data, str):
			try:
				# this assumes the first column contains timestamps so the next three columns are used
				# this also skips the first/header row 
				swim_data = np.genfromtxt(input_data,delimiter=',', skip_header=1, usecols=(1,2,3))
				swim_data= swim_data.astype(int)
			except (OSError, ValueError) as e:
				pass
				# raise IOError(f"Error reading CSV file: {str(e)}")
		else:
			raise ValueError("Invalid input type. Enter a NumPy array or a CSV filepath")
		mid_point= swim_data.shape[0] // 2
		swim_deg= self.encoder_to_degrees(swim_data)
		mirrored_data=self.degrees_to_encoder(-swim_deg)


		# creates an array to hold the encoder positions for all 12 motors
		motor_pos_shape = (swim_data.shape[0], 12)
		motor_pos= np.zeros(motor_pos_shape)
		# populates instruction array
		motor_pos[:, :3]= swim_data

		# makes it so that the diagonal legs are 180 degrees out of phase
		# two legs read the encoder instructions normallly
		# while the other two legs start reading halfway through the instruction set 
		# and then loop back to the beginning 
		# if there is an even number of rows
		if motor_pos.shape[0]%2 == 0:
			motor_pos[:mid_point, 3:6] = swim_data[mid_point:, :]
			motor_pos[mid_point:, 3:6] = swim_data[:mid_point, :]
			
			motor_pos[:mid_point, 6:9] = mirrored_data[mid_point:, :]
			motor_pos[mid_point:, 6:9] = mirrored_data[:mid_point, :]
		# if there is an odd number of rows 
		else:
			motor_pos[:mid_point, 3:6] = swim_data[mid_point+1:, :]
			motor_pos[mid_point+1:, 3:6] = swim_data[:mid_point, :]
			
			motor_pos[:mid_point, 6:9] = mirrored_data[mid_point+1:, :]
			motor_pos[mid_point+1:, 6:9] = mirrored_data[:mid_point, :]
		
		motor_pos[:, 9:]=mirrored_data

		motor_pos=motor_pos.astype(int)
		print(motor_pos_shape)
		self.zero_motors()
		cycle = 0
		while cycle <= cycles:
			self.move(motor_pos, mode="encoder")
			cycle = cycle + 1 

	def inst_phase_offset(cycle_percent):
				# checks the input_data data type 
		if isinstance(input_data, np.ndarray):
			if (input_data.shape[1]==3):
				swim_data=input_data
			else:
				raise ValueError("Input array is not the correct size. It should have the shape (numer_of_instructions,3).")
		elif isinstance(input_data, str):
			try:
				# this assumes the first column contains timestamps so the next three columns are used
				# this also skips the first/header row 
				swim_data = np.genfromtxt(input_data,delimiter=',', skip_header=1, usecols=(1,2,3))
				swim_data= swim_data.astype(int)
			except (OSError, ValueError) as e:
				pass
				# raise IOError(f"Error reading CSV file: {str(e)}")
		else:
			raise ValueError("Invalid input type. Enter a NumPy array or a CSV filepath")
		motor_pos_shape = (swim_data.shape[0], 12)
		inst_num= swim_data.shape[0]
		offset_start= math.floor(inst_num * cycle_percent)
		motor_pos= np.zeros(motor_pos_shape)
		swim_deg= self.encoder_to_degrees(swim_data)
		mirrored_data=self.degrees_to_encoder(-swim_deg)
		# populates instruction array
		# diagonal legs will be in phase
		motor_pos[:, :3]= swim_data
		motor_pos[:, 9:]=mirrored_data
		# the other set of diagonal legs will be offset based on the cycle_percent input
		motor_pos[:(inst_num - offset_start), 3:6] = swim_data[offset_start:, :]
		motor_pos[(inst_num - offset_start):, 3:6] = swim_data[:offset_start, :]
			
		motor_pos[:(inst_num - offset_start), 6:9] = mirrored_data[offset_start:, :]
		motor_pos[(inst_num - offset_start):, 6:9] = mirrored_data[:offset_start, :]
		return motor_pos

    # Old. has not been tested yet. Don't use. Use standup () instead  
	# def crawl_to_walk(self):
	# 	presentPositions = self.robot.sync_read("position")
	# 	pos_deg = [self.encoder_to_degrees(i) for i in presentPositions]
	# 	# extend legs out
	# 	motor_commands = self.generate_bridge(pos_deg,Robot.crawl_to_walk_transition, 150)
	# 	self.move(motor_commands,20)
	# 	# push up 
	# 	motor_commands = self.generate_bridge(Robot.crawl_to_walk_transition,Robot.standing_start, 150)
	# 	self.move(motor_commands,20)
	
	def crawl_to_walk(self):
		#first number is outward swing (when standing)
		#second number is front and back swing (when standing)
		# pose1 = np.array([0,90,0,90,30,90,0,-90,0,-90,-30,-90]) #front legs in front. Back legs forward at 30 deg
		

		# pose1 = np.array([90,-30,90,0,-90,0,-90,30,-90,0,90,0]) #back legs in back. Front legs bend back to 30 deg

		pose1 = np.array([0,90,0,0,-90,0,0,-90,0,0,90,0])
		
		pose2 = np.array([0,130,0,0,40,0,0,-130,0,0,-40,0]) #

		pose3 = np.array([0,100,0,0,40,0,0,-100,0,0,-40,0]) #
		
		pose4 = np.array([0,10,0,0,-10,0,0,-10,0,0,10,0]) #


		self.robot.enable_torque(self.servo_ids)
		self.robot.set_profile_velocity(20, self.robot.servo_ids)

		servo_locations = [self.degrees_to_encoder(x) for x in pose1]
		self.robot.sync_write(type = "position", dataDict = servo_locations)
		time.sleep(4)
		servo_locations2 = [self.degrees_to_encoder(x) for x in pose2]
		self.robot.sync_write(type = "position", dataDict = servo_locations2)
		time.sleep(4)
		servo_locations3 = [self.degrees_to_encoder(x) for x in pose3]
		self.robot.sync_write(type = "position", dataDict = servo_locations3)
		time.sleep(2)
		servo_locations4 = [self.degrees_to_encoder(x) for x in pose4]
		self.robot.sync_write(type = "position", dataDict = servo_locations4)


	def walk_to_crawl(self):
		presentPositions = self.robot.sync_read("position")
		pos_deg = [self.encoder_to_degrees(i) for i in presentPositions]

		# print(current)
		motor_commands = self.generate_bridge(pos_deg,Robot.crawling_start, 150)
		self.move(motor_commands,20)



	def enable_torque(self,servo_ids=None):
		if servo_ids == None:
			servo_ids = self.servo_ids
		self.robot.enable_torque(servo_ids)

	def disable_torque(self,servo_ids=None):
		if servo_ids == None:
			servo_ids = self.servo_ids
		self.robot.disable_torque(servo_ids)

	def reboot(self,servo_ids=None):
		if servo_ids == None:
			servo_ids = self.servo_ids
		self.robot.reboot(servo_ids)

	def read_degrees(self):
		pos_data = self.robot.sync_read(type="position")
		pos_deg = [self.encoder_to_degrees(i) for i in pos_data]
		deg= np.array(pos_deg)
		print(deg.astype(int))
		



	def move(self, motor_commands, velocity=None, mode="degrees", delay=None):
		if motor_commands.shape[1]==3:
			old_servo_ids=self.robot.servo_ids
			self.robot.servo_ids= list(range(0,3))
		if velocity == None:
			velocity = self.leg_speed
		if delay == None:
			delay = self.move_delay
		# self.robot.enable_torque(self.robot.servo_ids)
		ts_old = time.time()
		if mode=="degrees":  
			for pos in motor_commands:
				# print(pos)
				pos_enc = [self.degrees_to_encoder(i) for i in pos]
				# print(pos_enc)
				self.robot.sync_write(type="position", dataDict = pos_enc)
				time.sleep(delay)
		elif mode=="encoder":
			actual_encoder_values = []
			for pos in motor_commands:
				# pos_deg = [self.encoder_to_degrees(i) for i in pos]
				# deg = np.array(pos_deg)
				# print(deg.astype(int))
				pos_data = self.robot.sync_read(type="position")
				actual_encoder_values.append(pos_data)
				#print(pos_data)
				#print(pos)
				self.robot.sync_write(type="position", dataDict = pos)
				time.sleep(delay)
			actual_encoder_values= np.array(actual_encoder_values)
	
		if motor_commands.shape[1]==3:
			self.robot.servo_ids = old_servo_ids
                
		if Robot.READ_CURR == True:
			pres_curr = self.robot.sync_read(type="current")
			Robot.curr_data.append(pres_curr)
		if Robot.READ_POS == True:
			pres_pos = self.robot.sync_read(type="position")
			Robot.pos_data.append(pres_pos)
		if Robot.READ_CURR == True:
			pres_vel = self.robot.sync_read(type="velocity")
			Robot.vel_data.append(pres_vel)
		if Robot.READ_VOLT == True:
			pres_volt = self.robot.sync_read(type="voltage")
			Robot.volt_data.append(pres_volt)

			# time_data.append(time.time()-start_time)
			# ts_new = time.time()            
			# hz = 1/(ts_new - ts_old)
			# # print("Tranmission speed is",str(hz), " Hz")
			# # time.sleep(0.01)
			# ts_old = time.time()

	def zero_motors(self):
		self.robot.enable_torque(self.servo_ids)
		self.robot.set_profile_velocity(10, self.robot.servo_ids)
		servo_locations = [self.degrees_to_encoder(x) for x in Robot.standing_start]
		self.robot.sync_write(type = "position", dataDict = servo_locations)
		# t_end = time.time() + 2 #run for 20 sec
		# while time.time() < t_end:
			# dxl_comm_result = self.robot.PresentPositionSyncRead.txRxPacket()
			# if dxl_comm_result != Robot.COMM_SUCCESS:
			# 	print("%s" % self.robot.packetHandler.getTxRxResult(dxl_comm_result))
		    # # Check if groupsyncread data of Dynamixel#1 is available
			# dxl_getdata_result = self.robot.PresentPositionSyncRead.isAvailable(self.robot.servo_ids[0], self.robot.rDict["ADDR_PRESENT_POSITION"], self.robot.rDict["LEN_PRESENT_POSITION"])
			# if dxl_getdata_result != True:
			# 	print("[ID:%03d] groupSyncRead getdata failed" % self.robot.servo_ids[0])
			# 	quit()
			# dxl1_present_position = self.robot.PresentPositionSyncRead.getData(self.robot.servo_ids[0], self.robot.rDict["ADDR_PRESENT_POSITION"], self.robot.rDict["LEN_PRESENT_POSITION"])

		    # print("[ID:%03d]  PresPos:%03d\t" % (robot.servo_ids[0], dxl1_present_position))
		    # servo_locations = self.repeat_value(self.degrees_to_encoder(0),self.num_motors)
		    # result = [comp(x) for x in my_array]
			# servo_locations = [self.degrees_to_encoder(x) for x in Robot.standing_start]
		    # servo_locations[11] = 0
			# self.robot.sync_write(type = "position", dataDict = servo_locations)

	def actions_symmetry(self,change):
		length = len(change)
		half_length = int(length/2)
		mod = np.zeros(length)
		mod[:half_length] = change[:half_length]
		mod[half_length:] = - change[half_length:]
		return mod

	def degrees_to_encoder(self,deg):
		#return int((deg+Robot.settings["neutral_deg"])/(Robot.settings["deg_per_enc"]))
		return np.round((deg + Robot.settings["neutral_deg"])/ Robot.settings["deg_per_enc"]).astype(int)

	def encoder_to_degrees(self,enc):
		return enc*Robot.settings["deg_per_enc"] - Robot.settings["neutral_deg"]

	def repeat_value(self, value, repeat):
		return [value]*repeat


	def move_leg(self,leg,tar_degs,steps,current):
		# if leg == 'FL': #just so the angles are correct on the legs
		# 	tar_degs[0] = -tar_degs[0]
		# 	tar_degs[2] = -tar_degs[2]

		[roll,pitch,twist]=Robot.leg_motors[leg]
		target1 = copy(current)
		target1[roll] = tar_degs[0] #up
		target2 = copy(target1)
		target2[pitch] = tar_degs[1] #forward
		target3 = copy(target2)
		target3[roll] = tar_degs[2] #down

		action1 = np.linspace(current,target1,steps)
		action2 = np.linspace(target1,target2,steps)
		action3 = np.linspace(target2,target3,steps)

		actions = np.concatenate((action1, action2, action3),axis=0)
		return actions


	def shift(self,shift_deg,steps,current):
		target = copy(current)
		pitch_motors = [2,8,14,20]
		for i in pitch_motors:
			target[i] += shift_deg

		actions = np.linspace(current,target,steps)


		return actions


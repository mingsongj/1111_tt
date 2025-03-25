### Summary
new new 
source install/setup.bash
sudo usermod -a -G dialout admin
``
`ls -l /dev/ttyUSB*`sudo chmod 777 /dev/ttyUSB0
ros2 topic pub --once /arduino/command std_msgs/msg/String "data: 'i'"

ls -l /dev/ttyACM*
sudo chmod 777 /dev/ttyUSB1
sudo chmod 777 /dev/ttyACM0
ros2 topic pub --once /arduino/command std_msgs/msg/String "{data: 'i,5'}"

ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=realsense_stereo_rect,visual_slam interface_specs_file:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_visual_slam/quickstart_interface_specs.json base_frame:=camera_link camera_optical_frames:="['camera_infra1_optical_frame', 'camera_infra2_optical_frame']"

ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py enable_gyro:=true enable_accel:=true launch_fragments:=realsense_stereo_rect,visual_slam interface_specs_file:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_visual_slam/quickstart_interface_specs.json base_frame:=camera_link camera_optical_frames:="['camera_infra1_optical_frame', 'camera_infra2_optical_frame']"

0````````````````````````````````````````````````````````````````### How to use container and ros (depreciated)
This is only when we use this repo without using VSLAM
0. Following the https://github.com/dusty-nv/jetson-containers/blob/master/docs/setup.md to add docker to user group and increase swap size
1. Pull this repo in your `home` directory
1. In the workspace `turtle_ros`, run `docker/run.sh`, it will create a container named `turtle_noport`
1. Next time, you just need to run `docker/start.sh` to start the same container
2. Once the container starts, run VS-code, and go to `docker` extension, find the `turtle_noport`, right click to attach the current workspack. The container will install VS server, which takes a while for the first time.
2. run `ros2_build.sh` to build the ros2 packages, and source bash files `source install/setup.sh`. 

### How to run the container in the issa_rom_common 
 Jiefeng: 
 - Run script in `turtle_ros` named `run_isaac_ros.sh` that calls `run_dev.sh`to start the container
 - `build_isaac_ros.sh` will build the ros packages from the top of the 
 - NOTE: if the building of the package has a problem due to --symlink-install (which allows that you don't have to compile python file each time you modify it), you first need to remove all contents in your build folder first: `rm -r build/*`. And run `ros2_build.sh` again.  

### How to add code and packages to (turtle_ros):
1. Add python file in the srv folder of motor_srv, for example
2. Change setup.py to include new entry points
3. rebuild ros packages with `build_isaac_ros.sh` at the level of turtle_ros

### steps after ssh into jetson:
1. Set date to be correct: `sudo date --set="11 JAN 2024 10:59:00"`
2. Restart container: `cd ${ISAAC_ROS_WS}/src/isaac_ros_common &&  ./scripts/run_dev.sh`  
2.5 attaching is `cd ${ISAAC_ROS_WS}/src/isaac_ros_common &&  ./scripts/run_dev.sh`
3. Source the workspace: `source install/setup.bash`  I already added this line to the dockerfile startup, so don't need to explicitly call this
4. To start motor nodes: `ros2 run motor_srv motor_service`
5. start new window and redo step 2-3. Run new node: `ros2 run motor_srv arduino_node`
6. split new window and redo step 2-3. Run robot status and realsense all following 2-3.
7. Finally, run the walking/swimming/crawling modes

### How to run nodes 
1. NOTE: when running different nodes, you need to run from different terminals. Either in VScode , or open another terminal in your host machine environment and then run `sudo docker exec -it turtle_noport bash` to attach a the terminal to the running container and `cd turtle_ros` and `source install/setup.sh`.
1. Motor Server: this is the node that interacts directly with the dynamixel motors
  - run `ros2 run motor_srv motor_service` to start a motor service
  - also contains publisher on topic 'robot_status'
2. Motor Client: this node sends commands to motor server to ask dynamixels to do different gaits 
  - make sure to run `ros2 run motor_srv motor_client 'ready_walk'` to enable torques etc
  - run `ros2 run motor_srv motor_client 'walk' 5` to call walking gait for 5 cycles. 
  - for new bias term, do `ros2 run motor_srv motor_client 'walk' 5 0` 
  - for swimming: do `ros2 run motor_srv motor_client 'flap' 5 0`  (or `paddle`)
3. Robot Status Subscriber: does Cost of Transport Calculations.  
  - run `ros2 run motor_srv robot_status_sub` 
  - subscribes to topic 'robot status'
  - subscribes to topic 'power'
  - does COT calculation for the duration that robot is walking
4. Power sensor and Pneumatic control: measures power consumption with Arduino and power meter, and control the pneumatic system
  - run `ros2 run motor_srv arduino_node` to start the arduino node
  - publishes to topic 'power' and 'voltage'
  - pneumatics: `ros2 topic pub --once /pneumatic_command std_msgs/msg/String "data: 'j,10'"`
5. VSLAM: follow tutorial to install https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/main/docs/tutorial-realsense.md 
  - to launch node run `ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py`

  rosbag `rosbag record -O /rosbag/test.bag /power /visual_slam/tracking/vo_pose`
  
### instantenous COT
1. run `ros2 run motor_srv instant_cot`

### measure voltage of Jetson
1. run `sudo cat /sys/bus/i2c/drivers/ina3221/1-0040/hwmon/hwmon3/in1_input`
2. Note that Jetson operates on 5V (5000mV). You can't really measure the input voltage...

### Joystick
1. Make sure you have pulled the forked joystick_ros2 package on Billy's github
1. Give permission to joystick: `sudo chmod 777 /dev/input/*`
2. Run the node with `ros2 run joystick_ros2 joystick_ros2`
3. (to test:  the joystick message shows up in topic `ros2 topic echo /joy`)
4. (to test joystick connection to linux: `sudo apt install joystick jstest-gtk evtest` and run `evtest`)
5. Actual running the joystick client code: `ros2 run motor_srv joystick`
6. Make sure joystick is turned on. 
### Development plan 


#### penumatic code  in Arduino
Jiefeng:
 - Added `break` in the while loop of each subfunction, so that it will check if it needs to break out of the while loop as when you send a string "0"
 - Added `sensing` in the while loop of each subfunction (done)
 - Encoded the command type and the timeout for each command together eg., a string command: 'j50', means that the code will call  'jamming' function for 50 seconds. 
 - I will need to add a flag in the front of the power senser data so that we won't accidentally read the communication for the Pneumatic system.  
 
 Esteban: modify the arduino code to add sensor flag

### Utils folder
USB rules for US to distinguish the arduino and the motor controller. Follow the instruction bellow
 - creates symbolic names or aliases for USB devices based on specific product attributes
 - these device attibutes can be found using `lsusb` or `udevadm info -a -n /dev/ttyUSB0`
 - vender ID is on the left, product ID is on the rigth when you use lsusb
 - replace `/dev/ttyUSB0` with the name of the device you wish to inspect 
 - this file must be copied onto the jetson nano's `/etc/udev/rules.d/` directory
 - to implement these new rules, udevadm should be reset; use `sudo udevadm control --reload-rules`
 - then run `sudo udevadm trigger`
 - these USB device attributes are product specific, so new devices require updated rules 
 - you can check that these symbolic names are being mapped properly using using `udevadm monitor` that can show the pluging in rea-time
 - `ls -l /dev/ | grep USB` shows the maping
 - double check the attributes in your rules if this is outputting odd behavior
 - `find_device_path.py` looks for the symbolic names `ttyUSB_Arduino` and `ttyUSB_MotorControl`, so if they are changed that file must also be updated

Erick will need to write a code to directly return the correct USB port when when input `arduino` or `motor`

 
 - How to use it? Write it here:  (Erick modify the read me and use `` to make the code readable)
 Modified find_device_path.py and created update_udev_rules.py. Also modified motor_srv_func.py and arduino_node.py so that they utilize find_device_path.py
 If you run find_device_path.py from the terminal i.e. python3 find_device_path.py, this will retrieve the actual names for the symbolic names 'arduino' and 'motor' (this is just useful default behavior)
 The output of the function get_device_path (inside find_device_path.py) is the actual path of the symbolic name (as a string). This function takes in any symbolic name or any list of of symbolic names (strings in both cases). symlink_names= ["arduino", "motor"] and symlink_names= "motor" are both valid inputs. 
  import find_device_path
  path = find_device_path.get_device_path('arduino')
 Here path would equal /dev/ttyUSB1 
 I updated motor_srv_func.py and arduino_node.py so that they call find_device_path.get_device_path() using the symbolic names 'arduino' and 'motor'. But these are arbitrary and can be updated by adjusting the udev rules. 

 I created update_udev_rules.py to streamline updating the symbolic name rules. The commands inside this script that reset the udevadm rules need root user privileges, so to run this script you should use sudo python3 update_udev_rules.py from the terminal. Running this script from other python files (importing it into another .py file) won't work without first granting yourself root user privileges in jupyter notebook or vscode, but I saw that that is not recommended. 
 This script creates a 99-custom-usb.rules file if it doesn't already exist on your jetson. And writes the rules for assigning the symbolic names 'arduino' and 'motor', which are used in the scripts above. When you call this function from the terminal you should pass the arguments for the vendor and products ids of the devices that you want to make a rule for.
 sudo python3 update_rules.py --arduino_vendor_id 1234 --arduino_product_id 5678 --motor_vendor_id 4321 --motor_product_id 8765
 All of these arguments have default values (the motor vendor and product ids are from the U2D2 motor controller device and the arduino vendor and product values are from the elegoo arduino nano), so if you just run sudo python3 update_rules.py, the two rules should appear:
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="arduino"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", SYMLINK+="motor"
This script also checks that a rule for you specific device doesn't already exist before adding it. So a new rules is only added if the ids you provide are new. 
All of these are valid:

sudo python3 update_rules.py --arduino_vendor_id 1234 --arduino_product_id 5678 --motor_vendor_id 4321 --motor_product_id 8765
sudo python3 update_rules.py --arduino_vendor_id 1234 --arduino_product_id 5678
sudo python3 update_rules.py --motor_vendor_id 1234 --motor_product_id 5678
This script isn't too flexible. It only makes new rules for the symbolic names arduino and motor. So if you want to make a symbolic name for a different type of device, the script should be update, but this isn't a complicated fix. 
This script handles all the rule resetting and makes sure that everything is in the right directory!

### adding oled on the jetson. 
Erick:
Can you try to add an oled to show the ip address and which wifi, we are connecting to?


### Network and Wi-Fi
Luis, test the wifi. Can it work with out subscribing to the data?  

### Pneumatic system prepration
Esteban:
- Fix the wire kinking (We might need a larger syringe or tube? You will need to think about a solution.) (done)
- Find a glue or a new hoursing --- basically, a sollution for airtight the system without kinking wires. 
- Next version of electrical system to be even compact. get rid off most connectors, sockets and just leave 12V in and the output for the pumps and valves. The arduino and the motor drive can be directly solder on the board. 
- Make this version workable, and then we can build another version. 

### Experiments
Because, we 

### 
Some useful commands:
- docker container prune
- ros2 pkg create --build-type ament_python turtle_ros --dependencies rclpy example_interface dynamixel_sdk
- rosdep install -i --from-path src --rosdistro humble -y

### Notes
Arduino does not currently support nvidia jetson devices so you have to install this using a github repo, instructions are found on this website:
https://jetsonhacks.com/2019/10/04/install-arduino-ide-on-jetson-dev-kit/, basically, just run the following commands

```
git clone https://github.com/JetsonHacksNano/installArduinoIDE
cd installArduinoIDE
./installArduinoIDE.sh
```


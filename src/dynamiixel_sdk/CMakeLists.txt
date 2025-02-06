cmake_minimum_required(VERSION 3.5)
project(dynamixel_sdk)

# Find required ROS 2 packages and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable targets here if needed
# add_executable(your_executable_name src/your_source_file.cpp)

# Add dependencies to your executable if needed
# ament_target_dependencies(your_executable_name rclcpp std_msgs)

# Install Python scripts if you have any
# install(PROGRAMS scripts/your_python_script.py
#   DESTINATION lib/${PROJECT_NAME})

# Install Python launch files if you have any
# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch)

# Install package.xml
ament_package()

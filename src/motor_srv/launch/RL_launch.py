import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    visual_slam_pkg = get_package_share_directory('isaac_ros_visual_slam')
    return LaunchDescription([
        # Print message for starting goal_publisher
        LogInfo(msg="Starting 'goal_publisher' node..."),
        Node(
            package='motor_srv',
            executable='goal_publisher',
            name='goal_publisher',
            output='screen'
        ),
        
        # Print message for starting sync_read_write
        LogInfo(msg="Starting 'sync_read_write' node..."),
        Node(
            package='motor_srv',
            executable='sync_read_write',
            name='sync_read_write',
            output='screen'
        ),

        # Print message for starting keyboard_control_node (Uncomment if needed)
        # LogInfo(msg="Starting 'keyboard_control_node' node..."),
        # Node(
        #     package='motor_srv',
        #     executable='keyboard_control_node',
        #     name='keyboard_control_node',
        #     output='screen'
        # ),

        #Print message for starting keyboard_control_node (Uncomment if needed)
        LogInfo(msg="Starting 'robot_motion_tracker' node..."),
        Node(
            package='motor_srv',
            executable='robot_motion_tracker',
            name='robot_motion_tracker',
            output='screen'
        ),

        # # Print message for starting Isaac ROS Visual SLAM
        # LogInfo(msg="Starting Isaac ROS Visual SLAM..."),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         os.path.join(
        #             get_package_share_directory('isaac_ros_examples'),
        #             'launch',
        #             'isaac_ros_examples.launch.py'
        #         )
        #     ]),
        #     launch_arguments={
        #         'launch_fragments': 'realsense_stereo_rect,visual_slam',
        #         'interface_specs_file': f'{isaac_ros_ws}/isaac_ros_assets/isaac_ros_visual_slam/quickstart_interface_specs.json',
        #         'base_frame': 'camera_link',
        #         'camera_optical_frames': "['camera_infra1_optical_frame', 'camera_infra2_optical_frame']"
        #     }.items()
        # ),

        # # Print message for starting RealSense camera
        # LogInfo(msg="Starting RealSense camera with IMU..."),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         os.path.join(
        #             get_package_share_directory('realsense2_camera'),
        #             'launch',
        #             'rs_launch.py'
        #         )
        #     ]),
        #     launch_arguments={
        #         'enable_gyro': 'true',
        #         'enable_accel': 'true'
        #     }.items()
        # ),

        LogInfo(msg="Starting Isaac ROS Visual SLAM with RealSense..."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(visual_slam_pkg, 'launch', 'isaac_ros_visual_slam_realsense.launch.py')
            ),
            # If you need to pass arguments to the included launch file, add them here.
            # For example:
            # launch_arguments={
            #     'some_arg': 'value',
            #     'another_arg': 'value2'
            # }.items()
        ),

        # Final success message
        LogInfo(msg="All nodes and processes have been launched successfully!")
    ])

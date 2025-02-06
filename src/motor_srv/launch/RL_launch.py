from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo

def generate_launch_description():
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

        # # Print message for starting keyboard_control_node
        # LogInfo(msg="Starting 'keyboard_control_node' node..."),
        # Node(
        #     package='motor_srv',
        #     executable='keyboard_control_node',
        #     name='keyboard_control_node',
        #     output='screen'
        # ),

        # Print message for starting RealSense camera
        LogInfo(msg="Starting RealSense camera launch..."),
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
                'enable_gyro:=true', 'enable_accel:=true'
            ],
            output='screen'
        ),

        # Final success message
        LogInfo(msg="All nodes and processes have been launched successfully!")
    ])

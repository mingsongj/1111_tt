from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cycle', default_value='1', description='Cycle number for gaits'),
        DeclareLaunchArgument('devicename', default_value='/dev/ttyUSB0', description='Serial device name'),

        Node(
            package='motor_srv', 
            executable='motor_service',  
            name='motor_service',
            namespace='',
            output='screen',
            parameters=[
                {'cycle': LaunchConfiguration('cycle')},
                {'devicename': LaunchConfiguration('devicename')}
            ],
            remappings=[
                ('/robot_status', '/robot_status'),  # Modify topic remappings if needed
            ],
        ),

        LogInfo(
            condition=LaunchConfiguration('cycle') == '1',  # Log only if 'cycle' argument is set to 1
            msg="My node launched with cycle number: '{cycle}' and device name: '{devicename}'"
        )
    ])

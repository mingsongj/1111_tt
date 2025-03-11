import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([

        # Print message for starting COT_jue node
        LogInfo(msg="Starting 'COT_jue' node..."),
        Node(
            package='motor_srv',  # Adjust package name if different
            executable='COT_jue',  # Assuming COT_jue.py is the script name
            name='cot_jue',  
            output='screen'
        ),

        # Print message for starting RL_implement node
        LogInfo(msg="Starting 'RL_implementation' node..."),
        Node(
            package='motor_srv',  # Adjust package name if different
            executable='RL_implementation',  # Assuming RL_implement.py is the script name
            name='rl_implementation',
            output='screen'
        ),

        # Final success message
        LogInfo(msg="All nodes and processes have been launched successfully!")
    ])
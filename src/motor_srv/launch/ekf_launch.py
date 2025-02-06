from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Use the correct absolute path with a leading slash
    package_dir = '/workspaces/isaac_ros-dev/src/mingsong_turtle_try/src/motor_srv/config'
    
    # Ensure the YAML file path is correctly specified
    yaml_file_path = os.path.join(package_dir, 'ekf_config.yaml')

    # Verify the file existence before running
    if not os.path.isfile(yaml_file_path):
        raise FileNotFoundError(f"Configuration file not found: {yaml_file_path}")

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[yaml_file_path]
        )
    ])

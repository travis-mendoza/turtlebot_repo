from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('analog_reader')
    params_file = os.path.join(pkg_dir, 'config', 'analog_reader_params.yaml')
    
    return LaunchDescription([
        Node(
            package='analog_reader',
            executable='analog_reader_node',
            name='analog_reader_node',
            parameters=[params_file],
            output='screen'
        )
    ])
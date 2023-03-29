from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('donkey_car'), 
        'params', 
        'donkey_car_config.yaml')

    return LaunchDescription([
        Node(
            package='donkey_car',
            namespace='donkey_car',
            executable='donkey_car_node',
            name='donkey_car',
            parameters=[config_file]
        )
    ])

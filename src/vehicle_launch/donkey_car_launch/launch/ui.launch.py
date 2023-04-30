from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('donkey_car_launch'),
        'param',
        'ui.yaml'
    )
    return LaunchDescription([
        Node(
            package='donkey_car',
            namespace='donkey_car',
            executable='donkey_car_ui_node',
            name='donkey_car_ui_node',
            parameters=[config]
        )
    ])

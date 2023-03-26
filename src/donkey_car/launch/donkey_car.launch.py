from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='donkey_car',
            namespace='donkey_car',
            executable='donkey_car_node',
            name='donkey_car'
        )
    ])

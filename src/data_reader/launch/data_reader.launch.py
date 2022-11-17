from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='data_reader',
            namespace='data_reader',
            executable='data_reader_node',
            name='data_reader'
        )
    ])

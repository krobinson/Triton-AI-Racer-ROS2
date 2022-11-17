from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    data_read_config = os.path.join(
        get_package_share_directory('data_read'),
        'params',
        'data_read_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='data_read',
            namespace='data_read_namespace',
            executable='data_read_node',
            name='data_read_node',
            parameters=[data_read_config]
        )

    ])

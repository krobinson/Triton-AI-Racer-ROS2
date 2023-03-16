from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('donkeycar_ros'),
        'param',
        'train_car_data.yaml'
    )
    return LaunchDescription([
        Node(
            package='donkeycar_ros',
            executable='donkeycar_ros_train_node',
            name='donkeycar_ros_train',
            output='screen',
            emulate_tty=True,
            parameters=[
                config
            ]
        )

    ])
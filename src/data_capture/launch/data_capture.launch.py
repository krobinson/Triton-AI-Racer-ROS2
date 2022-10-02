from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    data_capture_config = os.path.join(
        get_package_share_directory('data_capture'), 
        'params', 
        'data_capture_config.yaml'
        )
    
    return LaunchDescription([
        Node(
            package='data_capture',
            namespace='waypoint',
            executable='image_and_control',
            name='data_capture',
            parameters=[data_capture_config]           
        )

   ])
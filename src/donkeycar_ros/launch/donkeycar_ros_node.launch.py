from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    data_capture_config = os.path.join(
        get_package_share_directory('donkeycar_ros'), 
            'params', 
            'donkeycar_ros_config.yaml'
        )
    
    return LaunchDescription([
        Node(
            package='donkeycar_ros.management',
            executable='donkeycar_ros_train_node',
            name='donkeycar_ros_train_node',
            parameters=[donkeycar_ros_train_config]           
        )

   ])
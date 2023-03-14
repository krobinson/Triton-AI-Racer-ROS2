import os
from pathlib import Path
from data_capture.data_constants import *
from sensor_msgs.msg._image import Image as PillowImage

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log

class DonkeyCarRos(Node):
    def __init__(self):
        # Initialize Node
        super().__init__("donkeycar_ros_node")
        self._logger.info("started donkeycar road node")


def main(args=None):
    rclpy.init(args=args)
    node = DonkeyCarRos()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(str(e))
        raise e
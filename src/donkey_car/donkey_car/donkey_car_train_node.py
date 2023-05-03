import os
from pathlib import Path
from data_capture.data_constants import *
from sensor_msgs.msg._image import Image as PillowImage

import rosbag2_py
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg._image import Image as SensorImage
from PIL import Image as PillowImage
from tai_interface.msg._vehicle_control import VehicleControl
from typing import Final
import numpy as np
import cv2
from cv_bridge import CvBridge


class DonkeyCarTrainNode(Node):
    def __init__(self):
        super().__init__("donkey_car_train_node")
        HELP_FRAMEWORK = 'the AI framework to use (tensorflow|pytorch). ' \
            'Defaults to config.DEFAULT_AI_FRAMEWORK'
        self.declare_parameter('tub')
        self.declare_parameter('model')
        self.declare_parameter('type')
        self.declare_parameter('config')
        self.declare_parameter('myconfig')
        self.declare_parameter('framework')
        self.declare_parameter('checkpoint')
        self.declare_parameter('transfer')
        self.declare_parameter('comment')


def main(args=None):
    rclpy.init(args=args)
    node = DonkeyCarTrainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(str(e))
        raise e

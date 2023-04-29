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


class DonkeyCarModelsNode(Node):
    def __init__(self):
        super().__init__("donkey_car_models_node")
        self.declare_parameter('models')


def main(args=None):
    rclpy.init(args=args)
    node = DonkeyCarModelsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(str(e))
        raise e

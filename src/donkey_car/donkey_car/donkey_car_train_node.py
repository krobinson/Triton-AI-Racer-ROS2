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
from cv_bridge import CvBridge
from donkeycar.management.base import Train

class DonkeyCarConfigAdapter():
    def __init__(self, 
                 default_model_type, 
                 batch_size, 
                 image_depth, 
                 image_h, 
                 image_w):
        self.DEFAULT_MODEL_TYPE = default_model_type
        self.BATCH_SIZE = batch_size
        self.IMAGE_DEPTH = image_depth
        self.IMAGE_H = image_h
        self.IMAGE_W = image_w
        self.VERBOSE_TRAIN = True
        self.PRINT_MODEL_SUMMARY = True


class DonkeyCarTrainNode(Node):
    def __init__(self):
        super().__init__("donkey_car_train_node")
        HELP_FRAMEWORK = 'the AI framework to use (tensorflow|pytorch). ' \
            'Defaults to config.DEFAULT_AI_FRAMEWORK'
        self.declare_parameter('tub', 'tub_path')
        self.declare_parameter('model', 'model_name')
        self.declare_parameter('type', 'type_name')
        self.declare_parameter('config', 'config_name')
        self.declare_parameter('myconfig', 'myconfig_name')
        self.declare_parameter('framework', 'pytorch')
        self.declare_parameter('checkpoint', 'checkpoint_name')
        self.declare_parameter('transfer', 'transfer_name')
        self.declare_parameter('comment', 'comment_name')
        self.get_parameter('myconfig')
        # get the necessary parameters

        HELP_FRAMEWORK = 'the AI framework to use (tensorflow|pytorch). ' \
            'Defaults to config.DEFAULT_AI_FRAMEWORK'
        tub_data = self.get_parameter('tub')
        output_model_name = self.get_parameter('model')
        model_type = self.get_parameter('type')
        config = self.get_parameter('config')
        my_config = self.get_parameter('myconfig')
        framework = self.get_parameter('framework')
        checkpoint = self.get_parameter('checkpoint')
        transfer_model = self.get_parameter('transfer')
        comment_dbase = self.get_parameter('comment')

        framework_str = framework.get_parameter_value().string_value
        model_type_str = model_type.get_parameter_value().string_value
        batch_size_int = 128
        image_depth_int = 3
        image_width_int = 320
        image_height_int = 240
        tub_data_path_str = tub_data.get_parameter_value().string_value
        output_model_name_str = output_model_name.get_parameter_value().string_value
        model_type_str = model_type.get_parameter_value().string_value
        checkpoint_path_str = checkpoint.get_parameter_value().string_value


        cfg = DonkeyCarConfigAdapter(model_type_str, 
                                    batch_size_int, 
                                    image_depth_int,
                                    image_width_int,
                                    image_height_int)

        if framework_str == 'pytorch':
            from donkeycar.parts.pytorch.torch_train import train
            train(cfg, tub_data_path_str, output_model_name_str, model_type_str)
        else:
            self._logger.error(f"Unrecognized framework: {framework}. Please specify "
                               f"one of 'tensorflow' or 'pytorch'")


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

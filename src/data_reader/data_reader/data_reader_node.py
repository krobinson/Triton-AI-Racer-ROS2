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
from data_reader.data_reader import train


DATA_DIR: Final[str] = 'DATA_DIR'
RESOURCES_PATH = Path(os.environ[DATA_DIR])
DATA_FILE: Final[str] = os.environ['DATA_FILE']


class ImageAndControlReading(Node):
    def __init__(self):
        # Initialize Node
        super().__init__("data_reader_node")
        self._logger.info("started data reading")
        # bag_path = str(RESOURCES_PATH + "/" + DATA_FILE)

        self.reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py._storage.StorageOptions(
            uri=STORAGE_OPTIONS_URI, storage_id=STORAGE_OPTIONS_ID)
        converter_options = rosbag2_py._storage.ConverterOptions(
            CONVERTOR_OPTIONS_PARAM_1, CONVERTOR_OPTIONS_PARAM_2)
        self.reader.open(storage_options, converter_options)

        topic_types = self.reader.get_all_topics_and_types()

        # Create a map for quicker lookup
        type_map = {
            topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

        # Set filter for topic of string type
        # storage_filter = rosbag2_py.StorageFilter(topics=['/topic'])
        # reader.set_filter(storage_filter)

        msg_counter = 0

        self.img_array = []

        while self.reader.has_next():
            (topic, data, t) = self.reader.read_next()
            msg_type = get_message(type_map[topic])
            # self._logger.info("Msg type is " + str(repr(msg_type)))
            msg = deserialize_message(data, msg_type)
            # self._logger.info("Msg is " + str(repr(msg)))
            self._logger.info(repr(type(msg)))
            self._logger.info(repr(msg.header))
            if isinstance(msg, SensorImage):
                self._logger.info(repr(msg.height))
                self._logger.info(repr(msg.width))
                self._logger.info(repr(msg.encoding))
                self._logger.info(repr(msg.is_bigendian))
                self._logger.info(repr(msg.step))
                self.img_array = self.load_image(msg)
            elif isinstance(msg, VehicleControl):
                self._logger.info(repr(msg.longitudinal_control_type))
                self._logger.info(repr(msg.throttle))
                self._logger.info(repr(msg.brake))
                self._logger.info(repr(msg.target_velocity))
                self._logger.info(repr(msg.lateral_control_type))
                self._logger.info(repr(msg.steering_openloop))
                self._logger.info(repr(msg.steering_rad))
            train()

    def load_image(self, img: SensorImage) -> np.ndarray:
        """
        :param string filename:     path to image file
        :param cfg:                 donkey config
        :return np.ndarray:         numpy uint8 image array
        """
        pil_image = self.load_pil_image(img)
        self._logger.info("Type is " + repr(type(pil_image)))

        if pil_image is None:
            return None

        img_arr = np.asarray(pil_image)

        # If the PIL image is greyscale, the np array will have shape (H, W)
        # Need to add a depth channel by expanding to (H, W, 1)
        # if cv_image
        #h, w = img_arr.shape[:2]
        # img_arr = img_arr.reshape(h, w, 1)

        self._logger.info("Type is " + repr(type(img_arr)))
        return img_arr

    def load_pil_image(self, img: SensorImage) -> PillowImage:
        """
            Return a pillow image for manipulation. Also handles resizing.

            Args:
                filename (string): path to the image file
                cfg (object): donkey configuration file

            Returns: a np.ndarray image.
        """

        try:
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(
                img, desired_encoding='passthrough')
    
            ######
            # convert from openCV2 to PIL. Notice the COLOR_BGR2RGB which means that 
            # the color is converted from BGR to RGB
            color_converted = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_image=PillowImage.fromarray(color_converted)
            if pil_image.height != cfg.IMAGE_H or pil_image.width != cfg.IMAGE_W:
                pil_image = pil_image.resize((cfg.IMAGE_W, cfg.IMAGE_H))

            if cfg.IMAGE_DEPTH == 1:
                pil_image = pil_image.convert('L')
            return pil_image

        except Exception as e:
            self._logger.info("Failed to load image")
            self._logger.info(e)
            return None


def main(args=None):
    rclpy.init(args=args)
    node = ImageAndControlReading()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    print('Hi from data_read.')


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(str(e))
        raise e

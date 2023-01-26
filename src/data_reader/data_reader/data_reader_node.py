import os
from pathlib import Path
from data_capture.data_constants import *

import rosbag2_py
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg._image import Image
from tai_interface.msg._vehicle_control import VehicleControl
from typing import Final


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

        while self.reader.has_next():
            (topic, data, t) = self.reader.read_next()
            msg_type = get_message(type_map[topic])
            #self._logger.info("Msg type is " + str(repr(msg_type)))
            msg = deserialize_message(data, msg_type)
            #self._logger.info("Msg is " + str(repr(msg)))
            self._logger.info(repr(type(msg)))
            self._logger.info(repr(msg.header))
            if isinstance(msg, Image):
                self._logger.info(repr(msg.height))
                self._logger.info(repr(msg.width))
                self._logger.info(repr(msg.encoding))
                self._logger.info(repr(msg.is_bigendian))
                self._logger.info(repr(msg.step))
            elif isinstance(msg, VehicleControl):
                self._logger.info(repr(msg.longitudinal_control_type))
                self._logger.info(repr(msg.throttle))
                self._logger.info(repr(msg.brake))
                self._logger.info(repr(msg.target_velocity))
                self._logger.info(repr(msg.lateral_control_type))
                self._logger.info(repr(msg.steering_openloop))
                self._logger.info(repr(msg.steering_rad))                


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

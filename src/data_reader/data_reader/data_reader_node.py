import os
from pathlib import Path
from data_capture.data_capture.data_constants import *

import rosbag2_py
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
from typing import Final


DATA_DIR: Final[str] = 'DATA_DIR'
RESOURCES_PATH = Path(os.environ[DATA_DIR])
DATA_FILE: Final[str] = 'DATA_FILE'


class ImageAndControlReading(Node):
    def __init__(self):
        # Initialize Node
        super().__init__("data_reader_node")
        self._logger.info("started data reading")
        bag_path = str(RESOURCES_PATH + "/" + DATA_FILE)

        self.reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py._storage.StorageOptions(
            uri=STORAGE_OPTIONS_URI, storage_id=STORAGE_OPTIONS_ID)
        converter_options = rosbag2_py._storage.ConverterOptions(
            CONVERTOR_OPTIONS_PARAM_1, CONVERTOR_OPTIONS_PARAM_2)
        reader.open(storage_options, converter_options)

        topic_types = reader.get_all_topics_and_types()

        # Create a map for quicker lookup
        type_map = {
            topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

        # Set filter for topic of string type
        #storage_filter = rosbag2_py.StorageFilter(topics=['/topic'])
        # reader.set_filter(storage_filter)

        msg_counter = 0

        while reader.has_next():
            (topic, data, t) = reader.read_next()
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)

            msg_counter += 1

        # No filter
        reader.reset_filter()

        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        msg_counter = 0

        while reader.has_next():
            (topic, data, t) = reader.read_next()
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)

        assert isinstance(msg, Log) or isinstance(msg, String)

        if isinstance(msg, String):
            assert msg.data == f'Hello, world! {msg_counter}'
            msg_counter += 1


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

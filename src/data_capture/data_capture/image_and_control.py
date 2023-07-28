import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tai_interface.msg import VehicleControl
from waypoint_interfaces.srv import TerminateWaypointLogging, ToggleWaypointLogging, GetWaypointLogState
from rclpy.qos import qos_profile_sensor_data
from message_filters import ApproximateTimeSynchronizer
from message_filters import Subscriber

import rosbag2_py
from rcl_interfaces.msg import Log
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg._image import Image as SensorImage
from PIL import Image as PillowImage
from tai_interface.msg._vehicle_control import VehicleControl
from typing import Final
import numpy as np
import cv2
from cv_bridge import CvBridge

from typing import Final
from data_constants import STORAGE_OPTIONS_URI
from data_constants import STORAGE_OPTIONS_ID

IMAGE_TOPIC: Final[str] = '/cam/front'
CONTROL_TOPIC: Final[str] = '/vehicle_cmd'

class ImageAndControlLogging(Node):
    def __init__(self):
        super().__init__("image_and_control")
        self.declare_parameter('image_and_control_logging_hz', 20)
        self.present_image = None
        self.present_control = None

        # initialize ros bag
        self.initialize_file_writer()
        
        # set up synchronizer
        self.image_sub = Subscriber(self, Image, IMAGE_TOPIC, qos_profile_sensor_data)
        self.control_sub = Subscriber(self, VehicleControl, CONTROL_TOPIC, qos_profile_sensor_data)
        self.tss = ApproximateTimeSynchronizer([self.image_sub, self.control_sub], queue_size=10, slop=0.5)
        self.tss.registerCallback(self.write_messages)

    def write_messages(self, image: Image, control: VehicleControl):
        self.present_image = image
        self.present_control = control
        print("New images have been collected.")
        self.writer.

    def initialize_file_writer(self):
        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py._storage.StorageOptions(uri=STORAGE_OPTIONS_URI, storage_id=STORAGE_OPTIONS_URI)
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)



    def topic_callback(self, msg):
        """
        This is writing messages to the sqlite file
        """
        if isinstance(msg, SensorImage):
            self.writer.write(
                IMAGE_TOPIC,
                serialize_message(msg),
                self.get_clock().now().nanoseconds)
        elif isinstance(msg, VehicleControl):
            self.writer.write(
                CONTROL_TOPIC,
                serialize_message(msg),
                self.get_clock().now().nanoseconds)



def main(args=None):
    rclpy.init(args=args)
    node = ImageAndControlLogging()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(str(e))
        raise e

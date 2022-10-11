import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tai_interface.msg import VehicleControl
from waypoint_interfaces.srv import TerminateWaypointLogging, ToggleWaypointLogging, GetWaypointLogState
from rclpy.qos import QoSPresetProfiles
from rclpy.qos import qos_profile_sensor_data
from message_filters import ApproximateTimeSynchronizer
from message_filters import Subscriber
import rosbag2_py
from rclpy.serialization import serialize_message

from typing import Final

IMAGE_TOPIC: Final[str] = '/cam/front'
CONTROL_TOPIC: Final[str] = '/vehicle_cmd'

class ImageAndControlLogging(Node):
    def __init__(self):
        # Initialize Node
        super().__init__("image_and_control")

        # Open a file to write the messages to
        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py._storage.StorageOptions(
            uri='image_and_control',
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)        

        # Keep the last image and control
        self.present_image = None
        self.present_control = None
        
        # Subscribe to images and vehicle control to be saved so we can learn from them.
        self.image_sub = Subscriber(self, Image, IMAGE_TOPIC, qos_profile=qos_profile_sensor_data)
        self.control_sub = Subscriber(self, VehicleControl, CONTROL_TOPIC, qos_profile=qos_profile_sensor_data)
        self.tss = ApproximateTimeSynchronizer([self.image_sub, self.control_sub], queue_size=10, slop=0.5)
        self.tss.registerCallback(self.write_topics)

    def write_topics(self, image_and_control):
        self._logger.debug("received image and control" + image_and_control)
        #self.writer.write(
        #    'chatter',
        #    serialize_message(image_and_control),
        #    self.get_clock().now().nanoseconds)



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

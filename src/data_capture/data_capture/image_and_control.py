import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tai_interface.msg import VehicleControl
from waypoint_interfaces.srv import TerminateWaypointLogging, ToggleWaypointLogging, GetWaypointLogState
from rclpy.qos import QoSPresetProfiles
from rclpy.qos import qos_profile_sensor_data
from message_filters import ApproximateTimeSynchronizer
from message_filters import Subscriber

from typing import Final

IMAGE_TOPIC: Final[str] = '/cam/front'
CONTROL_TOPIC: Final[str] = '/vehicle_cmd'

class ImageAndControlLogging(Node):
    def __init__(self):
        super().__init__("image_and_control")
        self.declare_parameter('image_and_control_logging_hz', 20)
        
        self.present_image = None
        self.present_control = None
        
        self.image_sub = Subscriber(self, Image, IMAGE_TOPIC, qos_profile=qos_profile_sensor_data)
        self.control_sub = Subscriber(self, VehicleControl, CONTROL_TOPIC, qos_profile=qos_profile_sensor_data)
        self.tss = ApproximateTimeSynchronizer([self.image_sub, self.control_sub], queue_size=10, slop=0.5)
        self.tss.registerCallback(self.print_topics)

    def print_topics(self, image: Image, control: VehicleControl):
        self.present_image = image
        self.present_control = control
        print("New images have been collected.")



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

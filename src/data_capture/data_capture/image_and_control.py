import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tai_interface.msg import VehicleControl
from waypoint_interfaces.srv import TerminateWaypointLogging, ToggleWaypointLogging, GetWaypointLogState
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
        #self.declare_parameter('default_save_file_name', 'image_and_control')
        #self.declare_parameter('js_terminate_log_button', 1)
        #self.declare_parameter('js_get_log_state_button', 2)
        #self.logging_hz_ = self.get_parameter(
        #    "image_and_control_logging_hz").get_parameter_value().integer_value
        #self.logger_ = WaypointLogger(self.logging_hz_, self)

        self.present_image = None
        self.present_control = None

        self.image_sub = Subscriber(IMAGE_TOPIC, Image)
        self.control_sub = Subscriber(CONTROL_TOPIC, VehicleControl)
        self.tss = ApproximateTimeSynchronizer([self.image_sub, self.control_sub], queue_size=10, slop=0.5)
        self.tss.registerCallback(self.print_topics)
        # Pubs N' Subs
        #self.odom_sub_ = self.create_subscription(
        #    Odometry, "odom", self.odom_callback_, qos_profile_sensor_data)
        #self.joy_sub_ = self.create_subscription(
        #    Joy, "/joy", self.joy_callback_, 1)

        # ROS parameters
        #self.default_file_name_ = self.get_parameter(
        #    "default_save_file_name").get_parameter_value().string_value
        #self.js_toggle_log_button_ = self.get_parameter(
        #    "js_toggle_log_button").get_parameter_value().integer_value
        #self.js_terminate_log_button = self.get_parameter(
        #    "js_terminate_log_button").get_parameter_value().integer_value
        #self.js_get_log_state_button = self.get_parameter(
        #    "js_get_log_state_button").get_parameter_value().integer_value

        # ROS services
        #self.toggle_log_srv_ = self.create_service(ToggleWaypointLogging, "/waypoint/toggle_log",
        #                                           self.toggle_log_callback_)
        #self.terminate_log_srv_ = self.create_service(TerminateWaypointLogging, "/waypoint/terminate_log",
        #                                              self.terminate_log_callback_)
        #self.get_state_srv_ = self.create_service(GetWaypointLogState, "/waypoint/get_log_state",
        #                                          self.get_state_callback_)

        # Class members
        #self.state_toggle_log_button_ = 0
        #self.state_terminate_log_button_ = 0
        #self.state_get_log_state_button_ = 0

    def print_topics(self, image: Image, control: VehicleControl):
        self.present_image = image
        self.present_control = control
        print("New images have been collected.")



def main():
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

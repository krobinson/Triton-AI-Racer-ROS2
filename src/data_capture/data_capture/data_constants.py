
from multiprocessing.util import Finalize
from typing import Final


IMAGE_TOPIC: Final[str] = '/cam/front'
IMAGE_TOPIC_MSG_TYPE: Final[str] = 'sensor_msgs/msg/Image'
CONTROL_TOPIC: Final[str] = '/vehicle_cmd'
CONTROL_TOPIC_MSG_TYPE: Final[str] = 'tai_interface/msg/VehicleControl'
STORAGE_OPTIONS_URI: Final[str] = 'image_and_control'
STORAGE_OPTIONS_ID: Final[str] = 'sqlite3'
STORAGE_OPTIONS_ID_MCAP: Final[str] = 'mcap'
CONVERTOR_OPTIONS_PARAM_1: Final[str] = ''
CONVERTOR_OPTIONS_PARAM_2: Final[str] = ''

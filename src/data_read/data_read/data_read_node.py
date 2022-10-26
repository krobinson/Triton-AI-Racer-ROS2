
import rosbag2_py
import rclpy


class ImageAndControlReading(Node):
    def __init__(self):
        # Initialize Node
        super().__init__("image_and_control_reading")
        self._logger.info("started image and control reading")
        # Open a file to write the messages to
        self.reader = rosbag2_py.SequentialReader
        storage_options = rosbag2_py._storage.StorageOptions(
            uri='image_and_control',
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.reader.open(storage_options, converter_options)


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

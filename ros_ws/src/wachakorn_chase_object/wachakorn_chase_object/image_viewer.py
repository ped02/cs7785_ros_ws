import rclpy
import rclpy.logging
from rclpy.node import Node

import numpy as np
import cv2 as cv

import rclpy.qos
from sensor_msgs.msg import CompressedImage


class ImageSubscriber(Node):
    def __init__(self):
        super(ImageSubscriber, self).__init__('image_viewer')

        self.image = None

        # Setup subscriber
        image_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=2,
        )

        self.image_subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.set_image,
            image_qos_profile,
        )

        # Setup window
        self.window_name = 'Image Viewer'
        cv.namedWindow(self.window_name, cv.WINDOW_GUI_NORMAL)

        window_update_period_sec = 1 / 20  # sec
        self.show_timer = self.create_timer(
            window_update_period_sec, self.display_image
        )

    def set_image(self, image_data: CompressedImage):
        # np_arr = np.fromstring(image_data.data, np.uint8)
        image_data_raw = np.array(image_data.data)
        self.image = cv.imdecode(image_data_raw, cv.IMREAD_UNCHANGED)

        # self.image = np.array(image_data.data)

        self.get_logger().info(f'Received image data: {self.image.shape}')

    def display_image(self):
        # self.get_logger().info('Display Image')
        if self.image is not None:
            self.get_logger().info('=======Display Image')
            cv.imshow(self.window_name, self.image)
            cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    print(cv.__version__)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    cv.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

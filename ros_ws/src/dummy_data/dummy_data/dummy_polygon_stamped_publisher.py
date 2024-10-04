import rclpy
from rclpy.node import Node, ParameterDescriptor

from std_msgs.msg import Header
from geometry_msgs.msg import Polygon, Point32

from geometry_msgs.msg import PolygonStamped


class DummyPolygonStampedPublisher(Node):
    def __init__(self, **kwargs):
        super(DummyPolygonStampedPublisher, self).__init__(
            'dummy_polygon_publisher', **kwargs
        )

        frame_id_parameter_descriptor = ParameterDescriptor(
            description='Target frame_id'
        )
        self.declare_parameter(
            'frame_id', 'offset', frame_id_parameter_descriptor
        )

        self.polygon_publisher = self.create_publisher(
            PolygonStamped, '/segmenter/image_bounding', 10
        )

        PUBLISH_PERIOD_SEC = 0.1
        self.counter = 0
        self.publish_timer = self.create_timer(
            PUBLISH_PERIOD_SEC, self.publish_polygon_array
        )

    def publish_polygon_array(self):
        target_frame_id = (
            self.get_parameter('frame_id').get_parameter_value().string_value
        )

        # 2 side plane
        # planes = [
        #     [(1,1,1), (1,-1,1), (1,-1,-1), (1,1,-1)],
        #     [(1,1,-1), (1,-1,-1), (-1,-1,-1), (-1,1,-1)]
        # ]
        bounding_boxes = [
            [(0, 0), (50, 0), (50, 30), (0, 30)],
            [(150, 100), (250, 100), (250, 200), (150, 200)],
            [(0, 0), (320, 0), (320, 240), (0, 240)],
        ]

        PERIOD_COUNT = 30

        self.polygon_publisher.publish(
            PolygonStamped(
                header=Header(
                    frame_id=target_frame_id,
                    stamp=self.get_clock().now().to_msg(),
                ),
                polygon=Polygon(
                    points=[
                        Point32(x=float(x), y=float(y))
                        for x, y in bounding_boxes[self.counter // PERIOD_COUNT]
                    ]
                ),
            )
        )

        self.counter = (self.counter + 1) % (PERIOD_COUNT * len(bounding_boxes))


def main(args=None):
    rclpy.init(args=args)

    dummy_polygon_publisher = DummyPolygonStampedPublisher()

    # Spin Node
    rclpy.spin(dummy_polygon_publisher)

    # Destroy node
    dummy_polygon_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

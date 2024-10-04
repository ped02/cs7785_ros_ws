import rclpy
from rclpy.node import Node, ParameterDescriptor

from std_msgs.msg import Header
from geometry_msgs.msg import Polygon, Point32

from robot_interfaces.msg import PolygonArrayStamped


def polygon_list_to_message(polygons, frame_id, timestamp):
    return PolygonArrayStamped(
        header=Header(frame_id=frame_id, stamp=timestamp),
        polygons=[
            Polygon(
                points=[
                    Point32(
                        x=float(point[0]), y=float(point[1]), z=float(point[2])
                    )
                    for point in polygon
                ]
            )
            for polygon in polygons
        ],
    )


class DummyPolygonArrayPublisher(Node):
    def __init__(self, **kwargs):
        super(DummyPolygonArrayPublisher, self).__init__(
            'dummy_polygon_publisher', **kwargs
        )

        frame_id_parameter_descriptor = ParameterDescriptor(
            description='Target frame_id'
        )
        self.declare_parameter(
            'frame_id', 'fixed', frame_id_parameter_descriptor
        )

        self.polygon_array_publisher = self.create_publisher(
            PolygonArrayStamped, '/dummy_polygon_array', 10
        )

        PUBLISH_PERIOD_SEC = 1
        self.publish_timer = self.create_timer(
            PUBLISH_PERIOD_SEC, self.publish_polygon_array
        )

    def publish_polygon_array(self):
        target_frame_id = (
            self.get_parameter('frame_id').get_parameter_value().string_value
        )

        # 2 side plane
        planes = [
            [(1, 1, 1), (1, -1, 1), (1, -1, -1), (1, 1, -1)],
            [(1, 1, -1), (1, -1, -1), (-1, -1, -1), (-1, 1, -1)],
        ]

        self.polygon_array_publisher.publish(
            polygon_list_to_message(
                planes, target_frame_id, self.get_clock().now().to_msg()
            )
        )


def main(args=None):
    rclpy.init(args=args)

    dummy_polygon_array_publisher = DummyPolygonArrayPublisher()

    # Spin Node
    rclpy.spin(dummy_polygon_array_publisher)

    # Destroy node
    dummy_polygon_array_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

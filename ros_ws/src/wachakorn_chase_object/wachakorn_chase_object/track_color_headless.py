import numpy as np
import cv2 as cv

from numbers import Number
from collections.abc import Collection, Mapping
from typing import Tuple, Callable, Optional

import rclpy
import rclpy.logging
import rclpy.qos
from rclpy.node import Node, Parameter, SetParametersResult

from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point

from cv_bridge import CvBridge

from robot_interfaces.msg import ImageDict


def rgb_2_bgr(color: np.ndarray):
    return color[::-1]


def segment_color(
    image: np.ndarray,
    target_color_bgr: np.ndarray,
    color_deviation_hsv: np.ndarray,
):
    """Segment Image based on color in hsv space
    Return boolean mask
    """

    image_hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    target_color = np.squeeze(
        cv.cvtColor(
            np.expand_dims(target_color_bgr.astype(np.uint8), axis=(0, 1)),
            cv.COLOR_BGR2HSV,
        ).astype(int)
    )

    # Negative bound
    color_negative_bound = target_color - color_deviation_hsv

    # Positive bound
    color_positive_bound = target_color + color_deviation_hsv

    # Hue Intervals
    # | ----- | --t-- | ------ |

    # Normalize hue angle to 0 - 180
    hue_negative_bound = color_negative_bound[0] % 180
    hue_positive_bound = color_positive_bound[0] % 180

    hue_mask = np.logical_and(
        hue_negative_bound <= image_hsv[..., 0],
        image_hsv[..., 0] < hue_positive_bound,
    )

    if hue_negative_bound > hue_positive_bound:
        hue_mask = np.logical_not(hue_mask)

    # Saturation and Value
    sv_negative_bound = np.clip(color_negative_bound[1:], 0, 256)
    sv_positive_bound = np.clip(color_positive_bound[1:], 0, 256)

    sv_mask = np.logical_and.reduce(
        np.logical_and(
            sv_negative_bound <= image_hsv[..., 1:],
            image_hsv[..., 1:] < sv_positive_bound,
        ),
        axis=-1,
    )

    return np.logical_and(sv_mask, hue_mask)


def get_contours(
    object_mask, min_contour_area: Number, max_contour_area: Number
):
    # Get Contours
    # contours, _ = cv.findContours(255*np.astype(object_mask, np.uint8), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contours, _ = cv.findContours(
        255 * object_mask.astype(np.uint8),
        cv.RETR_EXTERNAL,
        cv.CHAIN_APPROX_SIMPLE,
    )

    # Filter Contours and sort
    def get_contour_value(contour) -> None | float:
        # Return sort value

        # Filter by area
        area = cv.contourArea(contour)

        if min_contour_area > area or max_contour_area < area:
            return None

        return area

    filtered_contours = []

    for contour in contours:
        contour_value = get_contour_value(contour)

        if contour_value is None:
            continue

        filtered_contours.append((contour, contour_value))

    filtered_contours_sorted = sorted(
        filtered_contours, key=lambda x: x[1], reverse=True
    )

    return [pair[0] for pair in filtered_contours_sorted]


def get_contour_center(contour) -> Optional[Tuple[Number, Number]]:
    contour_moment = cv.moments(contour)

    if contour_moment['m00'] != 0:
        centroid_x = contour_moment['m10'] / contour_moment['m00']
        centroid_y = contour_moment['m01'] / contour_moment['m00']

        return centroid_x, centroid_y


def display_contours(
    frame,
    contours,
    copy=True,
    show_contour=True,
    show_bounding=False,
    show_centroid=False,
):
    if copy:
        frame = np.copy(frame)

    # Draw contours
    if show_contour:
        frame = cv.drawContours(frame, contours, -1, (255, 0, 0), 2)

    for contour in contours:
        if show_bounding:
            # Draw bounding Rectangles]
            rect_x, rect_y, rect_width, rect_height = cv.boundingRect(contour)
            frame = cv.rectangle(
                frame,
                (rect_x, rect_y),
                (rect_x + rect_width, rect_y + rect_height),
                (0, 255, 0),
                2,
            )

        # Draw centroid
        contour_moment = cv.moments(contour)
        centroid_x = int(contour_moment['m10'] / contour_moment['m00'])
        centroid_y = int(contour_moment['m01'] / contour_moment['m00'])

        centroid_center = (centroid_x, centroid_y)

        if show_centroid:
            frame = cv.circle(frame, centroid_center, 5, (0, 0, 255), 2)

            frame = cv.putText(
                frame,
                f'{centroid_center}',
                centroid_center,
                cv.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2,
            )

    return frame


class ImageSubscriber(Node):
    def __init__(self, node_name='track_color_headless', **kwargs) -> None:
        super(ImageSubscriber, self).__init__(node_name, **kwargs)

        PROCESS_TIMER_UPDATE_PERIOD_SEC = 0.001  # Max

        ## Parameters

        parameter_validator: Mapping[
            str, Callable[[Parameter], SetParametersResult]
        ] = {}

        # Segmentation Parameters
        target_color_rgb_parameter_descriptor = ParameterDescriptor(
            description='Target RGB for tracking as array of 3 elements of int in range [0,255]'
        )
        self.declare_parameter(
            'target_color_rgb',
            [0, 0, 0],
            descriptor=target_color_rgb_parameter_descriptor,
        )

        def validate_target_color_rgb(
            parameter: Parameter,
        ) -> SetParametersResult:
            if parameter.type_ == Parameter.Type.INTEGER_ARRAY:
                target_color_rgb = np.array(
                    parameter.get_parameter_value().integer_array_value,
                    dtype=int,
                )

                if len(target_color_rgb) != 3:
                    return SetParametersResult(
                        False,
                        f'Expected Array of size 3, got {len(target_color_rgb)}',
                    )

                if np.logical_or(
                    target_color_rgb < 0, target_color_rgb > 255
                ).any():
                    return SetParametersResult(
                        successful=False,
                        reason=f'Expected array to be in range [0, 255], but got {target_color_rgb}',
                    )

                return SetParametersResult(successful=True)
            else:
                return SetParametersResult(
                    successful=False,
                    reason=f'Expected Array of Integer, got {parameter.type_}',
                )

        parameter_validator['target_color_rgb'] = validate_target_color_rgb

        color_deviation_hsv_parameter_descriptor = ParameterDescriptor(
            description='Deviation from target in HSV space as array of 3 elements of int in range [[0, 90], [0, 127], [0,127]]'
        )
        self.declare_parameter(
            'color_deviation_hsv',
            [0, 0, 0],
            descriptor=color_deviation_hsv_parameter_descriptor,
        )

        def validate_color_deviation_hsv(
            parameter: Parameter,
        ) -> SetParametersResult:
            if parameter.type_ == Parameter.Type.INTEGER_ARRAY:
                color_deviation_hsv = np.array(
                    parameter.get_parameter_value().integer_array_value,
                    dtype=int,
                )

                if len(color_deviation_hsv) != 3:
                    return SetParametersResult(
                        False,
                        f'Expected Array of size 3, got {len(color_deviation_hsv)}',
                    )

                lower_bound = np.array([0, 0, 0], dtype=int)
                upper_bound = np.array([90, 127, 127], dtype=int)

                if np.logical_or(
                    color_deviation_hsv < lower_bound,
                    color_deviation_hsv > upper_bound,
                ).any():
                    return SetParametersResult(
                        successful=False,
                        reason=f'Expected array to be in range [[0, 90], [0, 127], [0,127]], but got {color_deviation_hsv}',
                    )

                return SetParametersResult(successful=True)
            else:
                return SetParametersResult(
                    successful=False,
                    reason=f'Expected Array of Integer, got {parameter.type_}',
                )

        parameter_validator[
            'color_deviation_rgb'
        ] = validate_color_deviation_hsv

        # Tracking Parameters
        track_min_contour_area_parameter_descriptor = ParameterDescriptor(
            description='Minimum contour area tracked (pixels^2)'
        )
        self.declare_parameter(
            'track_min_contour_area',
            5.0 * 5.0,
            track_min_contour_area_parameter_descriptor,
        )

        track_max_contour_area_parameter_descriptor = ParameterDescriptor(
            description='Maximum contour area tracked (pixels^2)'
        )
        self.declare_parameter(
            'track_max_contour_area',
            240 * 320 * 0.8,
            track_max_contour_area_parameter_descriptor,
        )

        # Process Parameters
        process_rate_parameter_descriptor = ParameterDescriptor(
            description=f'Frame process rate in Hz (positive real number). Max {1/PROCESS_TIMER_UPDATE_PERIOD_SEC:.2f} HZ'
        )
        self.declare_parameter(
            'process_rate_hz',
            10.0,
            descriptor=process_rate_parameter_descriptor,
        )

        def validate_process_rate_hz(
            parameter: Parameter,
        ) -> SetParametersResult:
            if parameter.type_ == Parameter.Type.DOUBLE:
                process_rate_hz = parameter.get_parameter_value().double_value

                if process_rate_hz <= 0.0:
                    return SetParametersResult(
                        False,
                        f'Expected postive number, got {len(process_rate_hz)}',
                    )

                return SetParametersResult(successful=True)
            else:
                return SetParametersResult(
                    successful=False,
                    reason=f'Expected Double, got {parameter.type_}',
                )

        parameter_validator['process_rate_hz'] = validate_process_rate_hz

        # Display Tracking Result
        display_tracking_parameter_descriptor = ParameterDescriptor(
            description='Whether to publish tracking result. Bool'
        )
        self.declare_parameter(
            'display_tracking', False, display_tracking_parameter_descriptor
        )

        def validate_parameters(
            parameters: Collection[Parameter],
        ) -> SetParametersResult:
            for parameter in parameters:
                validator = parameter_validator.get(parameter.name, None)

                if validator:
                    validate_result = validator(parameter)
                    if not validate_result.successful:
                        return validate_result

            return SetParametersResult(successful=True)

        self.add_on_set_parameters_callback(validate_parameters)

        self.update_parameters()

        # Setup subscriber
        self.image = None
        self.image_processed = False

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

        # cv bridge
        self.bridge = CvBridge()

        # Setup Publisher
        publish_image_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.segment_publisher = self.create_publisher(
            ImageDict, '/segmenter/images', publish_image_qos_profile
        )

        self.tracking_publisher = self.create_publisher(
            CompressedImage,
            '/segmenter/tracking_image/compressed',
            publish_image_qos_profile,
        )

        self.point_publisher = self.create_publisher(Point, 'image_point', 2)

        # Process timer
        self.last_process_time = self.get_clock().now()
        self.process_timer = self.create_timer(
            PROCESS_TIMER_UPDATE_PERIOD_SEC, self.process_image
        )

    def set_image(self, image_data: CompressedImage):
        self.image = self.bridge.compressed_imgmsg_to_cv2(image_data, 'bgr8')

        self.get_logger().info(f'Received image data: {self.image.shape}')

    def update_parameters(self) -> None:
        self.target_color_bgr = rgb_2_bgr(
            np.array(
                self.get_parameter('target_color_rgb')
                .get_parameter_value()
                ._integer_array_value,
                dtype=int,
            )
        )
        self.color_deviation_hsv = np.array(
            self.get_parameter('color_deviation_hsv')
            .get_parameter_value()
            ._integer_array_value,
            dtype=int,
        )

        self.track_min_contour_area = (
            self.get_parameter('track_min_contour_area')
            .get_parameter_value()
            ._double_value
        )
        self.track_max_contour_area = (
            self.get_parameter('track_max_contour_area')
            .get_parameter_value()
            ._double_value
        )

        # Assuming not 0
        self.process_period_sec = (
            1.0
            / self.get_parameter('process_rate_hz')
            .get_parameter_value()
            ._double_value
        )

        self.display_tracking = (
            self.get_parameter('display_tracking')
            .get_parameter_value()
            .bool_value
        )

    def process_image(self) -> None:
        # Update Parameter after next process

        current_time = self.get_clock().now()
        if (
            current_time - self.last_process_time
        ).nanoseconds * 1e-9 < self.process_period_sec:
            return

        self.last_process_time = current_time

        self.update_parameters()

        # Don't Process Duplicate Frames
        if self.image is None or self.image_processed:
            return

        blur_frame = cv.GaussianBlur(self.image, (5, 5), 1)

        # self.get_logger().info(f'Target Color: {self.target_color_bgr},{type(self.target_color_bgr)} Color Deviation: {self.color_deviation_hsv},{type(self.color_deviation_hsv)}')

        # Get Segment Mask
        color_mask = segment_color(
            blur_frame, self.target_color_bgr, self.color_deviation_hsv
        )

        image_dict = ImageDict()

        image_dict.names.append('image')
        image_dict.images.append(
            self.bridge.cv2_to_compressed_imgmsg(self.image)
        )

        image_dict.names.append('segment_mask')
        image_dict.images.append(
            self.bridge.cv2_to_compressed_imgmsg(
                255 * color_mask.astype(np.uint8)
            )
        )

        self.segment_publisher.publish(image_dict)

        # Get Track Object
        object_contours = get_contours(
            color_mask, self.track_min_contour_area, self.track_max_contour_area
        )

        if len(object_contours) > 0:
            # Found object
            largest_center = get_contour_center(object_contours[0])
            self.point_publisher.publish(
                Point(x=largest_center[0], y=largest_center[1])
            )

        # Display Result
        if self.display_tracking:
            contour_frame = display_contours(
                self.image,
                object_contours,
                copy=False,
                show_bounding=True,
                show_centroid=True,
            )
            self.tracking_publisher.publish(
                self.bridge.cv2_to_compressed_imgmsg(contour_frame)
            )


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

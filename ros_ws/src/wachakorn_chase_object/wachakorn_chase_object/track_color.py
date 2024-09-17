from typing import Optional

import numpy as np
import cv2 as cv

import rclpy
import rclpy.logging
import rclpy.qos
from rclpy.node import Node

from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage


class ColorSelectorWindow:
    def __init__(
        self, window_name: str = 'Color Selector', average_width: int = 3
    ):
        if average_width % 2 != 1:
            raise ValueError(
                f'Average width must be odd, but is {average_width}'
            )

        # Window
        self.window_name = window_name

        cv.namedWindow(self.window_name, cv.WINDOW_GUI_NORMAL)
        cv.setMouseCallback(self.window_name, self.mouse_callback)

        self.frame = None

        # Select Color
        self.selected_color = None
        self.average_half_width = average_width // 2

        # Click
        self.mouse_down = False

    def get_selected_color(self) -> Optional[np.ndarray]:
        return self.selected_color

    def show_frame(self, frame):
        self.frame = frame
        cv.imshow(self.window_name, self.frame)

    def mouse_callback(self, event, x, y, flags, params):
        if self.frame is None:
            return

        if event == cv.EVENT_LBUTTONDOWN:
            # logger.info(f'Mouse location [x,y]: ({x},{y})')

            if not self.mouse_down:
                # Click color
                frame_height, frame_width = self.frame.shape[:2]

                lower_bound_x = max(0, x - self.average_half_width)
                lower_bound_y = max(0, y - self.average_half_width)

                upper_bound_x = min(
                    frame_width, x + self.average_half_width + 1
                )
                upper_bound_y = min(
                    frame_height, y + self.average_half_width + 1
                )

                self.selected_color = np.mean(
                    self.frame[
                        lower_bound_y:upper_bound_y, lower_bound_x:upper_bound_x
                    ],
                    axis=(0, 1),
                ).astype(np.uint8)

                rclpy.logging.get_logger().info(
                    f'Color selected [BGR]: {self.selected_color} from [{lower_bound_x}, {lower_bound_y}], [{upper_bound_x},{upper_bound_y}]'
                )

            self.mouse_down = True

        elif event == cv.EVENT_LBUTTONUP:
            self.mouse_down = False


def segment_color(
    image: np.ndarray,
    target_color_bgr: np.ndarray,
    color_deviation_hsv: np.ndarray,
):
    """Segment Image based on color in hsv space
    Return mask
    """

    image_hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    target_color = np.squeeze(
        cv.cvtColor(
            np.expand_dims(target_color_bgr, axis=(0, 1)), cv.COLOR_BGR2HSV
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


class ObjectTracker:
    def __init__(self):
        pass

        self.min_contour_area = 5 * 5
        self.max_contour_area = 240 * 320 * 0.8

    def get_contours(self, object_mask):
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

            if self.min_contour_area > area or self.max_contour_area < area:
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

    def display(
        self,
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
                rect_x, rect_y, rect_width, rect_height = cv.boundingRect(
                    contour
                )
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


class SegmentWindow:
    def __init__(self, window_name: str = 'Segment'):
        # Window
        self.window_name = window_name

        cv.namedWindow(self.window_name, cv.WINDOW_GUI_NORMAL)

        cv.createTrackbar(
            'Hue Deviation', self.window_name, 0, 180, self.hue_callback
        )
        cv.createTrackbar(
            'Saturation Deviation',
            self.window_name,
            0,
            255,
            self.saturation_callback,
        )
        cv.createTrackbar(
            'Value Deviation', self.window_name, 0, 255, self.value_callback
        )

        self.hue_deviation = 0
        self.saturation_deviation = 0
        self.value_deviation = 0

        self.object_tracker = ObjectTracker()

    def hue_callback(self, hue_deviation):
        self.hue_deviation = hue_deviation

    def saturation_callback(self, saturation_deviation):
        self.saturation_deviation = saturation_deviation

    def value_callback(self, value_deviation):
        self.value_deviation = value_deviation

    def process_frame(self, frame: np.ndarray, target_color: np.ndarray):
        color_deviation = np.array(
            [
                self.hue_deviation,
                self.saturation_deviation,
                self.value_deviation,
            ]
        )

        # Segment Color
        color_mask = segment_color(frame, target_color, color_deviation)

        # Get Contour
        object_contours = self.object_tracker.get_contours(color_mask)

        # Display
        contour_frame = self.object_tracker.display(
            frame,
            object_contours,
            copy=True,
            show_contour=True,
            show_bounding=True,
            show_centroid=True,
        )

        return color_mask, object_contours, contour_frame

    def show_frame(self, frame: np.ndarray, target_color: np.ndarray):
        color_mask, object_contours, contour_frame = self.process_frame(
            frame, target_color
        )
        color_mask_display = np.tile(
            np.expand_dims(255 * color_mask.astype(np.uint8), -1), (1, 1, 3)
        )

        display_frame = np.concatenate(
            [color_mask_display, contour_frame], axis=1
        )

        cv.imshow(self.window_name, display_frame)

        if len(object_contours) > 0:
            return object_contours[0]
        return None


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

        # Setup color selector
        self.color_selector_window = ColorSelectorWindow()

        # Setup segmenter window
        self.segment_window = SegmentWindow()

        image_windows_update = 1 / 10  # sec
        self.image_windows_timer = self.create_timer(
            image_windows_update, self.process_image
        )

        # Setup publisher
        self.point_publisher = self.create_publisher(Point, 'image_point', 2)

    def set_image(self, image_data: CompressedImage):
        # np_arr = np.fromstring(image_data.data, np.uint8)
        image_data_raw = np.array(image_data.data)
        self.image = cv.imdecode(image_data_raw, cv.IMREAD_UNCHANGED)

        # self.image = np.array(image_data.data)

        self.get_logger().info(f'Received image data: {self.image.shape}')

    def process_image(self):
        if self.image is not None:
            self.color_selector_window.show_frame(self.image)

            if self.color_selector_window.selected_color is not None:
                # Blur Frame
                blur_frame = cv.GaussianBlur(self.image, (5, 5), 1)

                largest_object = self.segment_window.show_frame(
                    blur_frame, self.color_selector_window.selected_color
                )

                if largest_object is not None:
                    object_contour = largest_object
                    contour_moment = cv.moments(object_contour)

                    if contour_moment['m00'] != 0:
                        centroid_x = (
                            contour_moment['m10'] / contour_moment['m00']
                        )
                        centroid_y = (
                            contour_moment['m01'] / contour_moment['m00']
                        )

                        object_center = Point(x=centroid_x, y=centroid_y)

                        self.point_publisher.publish(object_center)

            cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    print(np.__version__)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    cv.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

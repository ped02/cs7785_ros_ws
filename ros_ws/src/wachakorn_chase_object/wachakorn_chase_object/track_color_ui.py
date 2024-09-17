import numpy as np
import cv2 as cv

from collections.abc import Mapping
from typing import Optional, Tuple
import abc

import rclpy
import rclpy.exceptions
import rclpy.executors
import rclpy.logging
import rclpy.qos
from rclpy.node import Node

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

from cv_bridge import CvBridge

from robot_interfaces.msg import ImageDict


class IShowFrame:
    @abc.abstractmethod
    def show_frame(self, frame: np.ndarray):
        raise NotImplementedError('Please Implement this method')


class SettingWindow(IShowFrame):
    def __init__(self, window_name='Track Color UI', average_width: int = 3):
        self.window_name = window_name

        cv.namedWindow(self.window_name, cv.WINDOW_GUI_NORMAL)

        self.frame = None

        # Color Selection
        cv.setMouseCallback(self.window_name, self.mouse_callback)

        self.selected_color_brg = np.array([0, 0, 0], dtype=int)
        self.average_half_width = average_width // 2

        # Click
        self.mouse_down = False

        # Deviation
        cv.createTrackbar(
            'Hue Deviation', self.window_name, 0, 90, self.hue_callback
        )
        cv.createTrackbar(
            'Saturation Deviation',
            self.window_name,
            0,
            127,
            self.saturation_callback,
        )
        cv.createTrackbar(
            'Value Deviation', self.window_name, 0, 127, self.value_callback
        )

        self.hue_deviation = 0
        self.saturation_deviation = 0
        self.value_deviation = 0

        # Parameter Changed
        self.parameter_changed = False

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

                self.selected_color_brg = np.mean(
                    self.frame[
                        lower_bound_y:upper_bound_y, lower_bound_x:upper_bound_x
                    ],
                    axis=(0, 1),
                ).astype(np.uint8)
                self.parameter_changed = True
                rclpy.logging.get_logger('track_color_ui').info(
                    f'Color selected [BGR]: {self.selected_color_brg} from [{lower_bound_x}, {lower_bound_y}], [{upper_bound_x},{upper_bound_y}]'
                )

            self.mouse_down = True

        elif event == cv.EVENT_LBUTTONUP:
            self.mouse_down = False

    def hue_callback(self, hue_deviation):
        self.hue_deviation = hue_deviation
        self.parameter_changed = True

    def saturation_callback(self, saturation_deviation):
        self.saturation_deviation = saturation_deviation
        self.parameter_changed = True

    def value_callback(self, value_deviation):
        self.value_deviation = value_deviation
        self.parameter_changed = True

    def _get_target_color(self) -> Optional[np.ndarray]:
        return self.selected_color_brg

    def _get_color_deviation(self) -> np.ndarray:
        return np.array(
            [
                self.hue_deviation,
                self.saturation_deviation,
                self.value_deviation,
            ],
            dtype=int,
        )

    def get_parameters(self) -> Tuple[np.ndarray, np.ndarray]:
        """Returns tuple (target_color_rgb, color_deviation_hsv)"""
        self.parameter_changed = False
        return self._get_target_color()[::-1], self._get_color_deviation()

    def get_parameter_changed(self) -> bool:
        return self.parameter_changed

    def show_frame(self, frame: np.ndarray):
        self.frame = frame
        cv.imshow(self.window_name, frame)


class ResultWindow:
    def __init__(self, window_name='Segment Result'):
        self.window_name = window_name
        cv.namedWindow(self.window_name, cv.WINDOW_GUI_NORMAL)

    def show_frame(self, frame: np.ndarray):
        cv.imshow(self.window_name, frame)


class TrackColorUI(Node):
    def __init__(self, node_name='track_color_ui', **kwargs):
        super(TrackColorUI, self).__init__(node_name, **kwargs)

        self.setting_window = SettingWindow()
        self.result_window = ResultWindow()

        # Subscription
        image_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=2,
        )

        self.images_subscriber = self.create_subscription(
            ImageDict,
            '/segmenter/images',
            self.receive_images,
            image_qos_profile,
        )

        # Update Parameter
        self.track_color_set_parameters_client = self.create_client(
            SetParameters, '/track_color_headless/set_parameters'
        )

        PARAMETER_UPDATE_PERIOD_SEC = 0.1
        self.updating = False
        self.update_timer = self.create_timer(
            PARAMETER_UPDATE_PERIOD_SEC, self.update_parameters
        )

        self.get_logger().info('Launched')

        self.bridge = CvBridge()

    def update_parameters(self):
        if self.updating:
            return

        if not self.setting_window.get_parameter_changed():
            return

        if not self.track_color_set_parameters_client.wait_for_service(
            timeout_sec=5.0
        ):
            self.get_logger().warn('Parameter set service not available')
            return

        (
            target_color_rgb,
            color_deviation_hsv,
        ) = self.setting_window.get_parameters()

        target_color_rgb_parameter = Parameter(
            name='target_color_rgb',
            value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER_ARRAY,
                integer_array_value=list(target_color_rgb),
            ),
        )

        color_deviation_hsv_parameter = Parameter(
            name='color_deviation_hsv',
            value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER_ARRAY,
                integer_array_value=list(color_deviation_hsv),
            ),
        )

        set_parameter_request = SetParameters.Request(
            parameters=[
                target_color_rgb_parameter,
                color_deviation_hsv_parameter,
            ]
        )
        future = self.track_color_set_parameters_client.call_async(
            set_parameter_request
        )

        future.add_done_callback(self.set_parameters_callback)

        self.updating = True

        self.get_logger().info('Requested setting parameter')

    def set_parameters_callback(self, future: rclpy.Future):
        self.updating = False
        try:
            response = future.result()

            if response:
                set_parameter_results = response.results
                all_success = True
                for set_parameter_result in set_parameter_results:
                    if not set_parameter_result.successful:
                        self.get_logger().error(
                            f'Update Failed: {set_parameter_result.reason}'
                        )
                        all_success = False

                if all_success:
                    self.get_logger().info('Parameter set successful')

            else:
                self.get_logger().error('Failed to update parmeters')

        except rclpy.executors.ExternalShutdownException as e:
            self.get_logger().error(f'ROS System Interrupted or Shutdown. {e}')

        except rclpy.executors.TimeoutException as e:
            self.get_logger().error(f'Service Timeout. {e}')

        except Exception as e:
            self.get_logger().erorr(f'Unexpected excepion. {e}')

    def receive_images(self, image_dict: ImageDict):
        window_map: Mapping[str, IShowFrame] = {
            'image': self.setting_window,
            'segment_mask': self.result_window,
        }

        # self.get_logger().info(f'Received')

        shown = False
        for k, image in zip(image_dict.names, image_dict.images):
            if k in window_map:
                window_map[k].show_frame(
                    self.bridge.compressed_imgmsg_to_cv2(image)
                )
                shown = True

        if shown:
            cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    track_color_ui = TrackColorUI()

    rclpy.spin(track_color_ui)

    track_color_ui.destroy_node()
    cv.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# Rachanon Wachakorn
import time
import datetime
import argparse
import os
import logging
from typing import Optional

import numpy as np

import cv2 as cv

logger = None


def setup_logger(verbose=False):
    global logger
    logger = logging.getLogger(__name__)
    stream_handler = logging.StreamHandler()
    logger.addHandler(stream_handler)
    if not verbose:
        logger.setLevel(logging.INFO)
    else:
        logger.setLevel(logging.DEBUG)


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

                self.selected_color = np.astype(
                    np.mean(
                        self.frame[
                            lower_bound_y:upper_bound_y,
                            lower_bound_x:upper_bound_x,
                        ],
                        axis=(0, 1),
                    ),
                    np.uint8,
                )

                logger.info(
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
        np.astype(
            cv.cvtColor(
                np.expand_dims(target_color_bgr, axis=(0, 1)), cv.COLOR_BGR2HSV
            ),
            int,
        )
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

        self.min_contour_area = 15 * 15
        self.max_contour_area = 480 * 640 * 0.8

    def get_contours(self, object_mask):
        # Get Contours
        # contours, _ = cv.findContours(255*np.astype(object_mask, np.uint8), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        contours, _ = cv.findContours(
            255 * np.astype(object_mask, np.uint8),
            cv.RETR_EXTERNAL,
            cv.CHAIN_APPROX_SIMPLE,
        )

        # Filter Contours and sort
        def get_contour_value(contour) -> bool:
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

        filtered_contours_sorted = sorted(filtered_contours, key=lambda x: x[1])

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

        return color_mask, contour_frame

    def show_frame(self, frame: np.ndarray, target_color: np.ndarray):
        color_mask, contour_frame = self.process_frame(frame, target_color)
        color_mask_display = np.tile(
            np.expand_dims(255 * np.astype(color_mask, np.uint8), -1), (1, 1, 3)
        )

        display_frame = np.concatenate(
            [color_mask_display, contour_frame], axis=1
        )

        cv.imshow(self.window_name, display_frame)


def main(video_url: str, frame_fetch_n: int, save_dir: str = ''):
    color_selector_window = ColorSelectorWindow()
    segment_window = SegmentWindow()

    camera = cv.VideoCapture(video_url)

    paused = False

    while True:
        if not paused:
            # Capture frame-by-frame
            capture_start_time = time.time()

            for _ in range(frame_fetch_n):
                camera.grab()
            ret, frame = camera.retrieve()

            if not ret:
                break

            capture_end_time = time.time()

            logger.info(
                f'Frame Time: {capture_end_time - capture_start_time} sec'
            )

            # Process
            color_selector_window.show_frame(frame)

            if color_selector_window.selected_color is not None:
                # Blur Frame
                blur_frame = cv.GaussianBlur(frame, (5, 5), 1)

                segment_window.show_frame(
                    blur_frame, color_selector_window.selected_color
                )

            # Display the resulting frame
            # cv.imshow('frame', frame)

        key = cv.waitKey(1)

        if key & 0xFF == ord('q'):
            # Exit
            break

        elif key & 0xFF == ord(' '):
            # Save image
            image_filename = f'screenshot_{datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")}.png'

            image_file_path = (
                os.path.join(save_dir, image_filename)
                if save_dir.strip() != ''
                else image_filename
            )

            cv.imwrite(image_file_path, frame)
            logger.debug(f'Saved image at: {image_file_path}')

        elif key & 0xFF == ord('p'):
            paused = not paused

    # When everything done, release the capture
    camera.release()
    cv.destroyAllWindows()


if __name__ == '__main__':
    setup_logger()

    parser = argparse.ArgumentParser(
        description='Video Viewer for file and url. Q to quit, space to screenshot, p to pause'
    )
    parser.add_argument(
        '--path', default=0, help='Path or URL to video resource'
    )
    parser.add_argument(
        '--save_dir', type=str, default='', help='Directory to save shots at'
    )
    parser.add_argument(
        '--frame_fetch_n',
        type=int,
        default=1,
        help='Amount of frame to read before retrieving',
    )

    args = parser.parse_args()

    logger.info('Video Viewer')
    logger.info("'q' to Quit")
    logger.info('Space to screenshot')
    logger.info("'p' to pause")

    main(args.path, args.frame_fetch_n, args.save_dir)

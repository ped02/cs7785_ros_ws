import time
import datetime
import argparse
import os
import logging


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


def main(video_url: str, frame_fetch_n: int, save_dir: str = ''):
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

            # Display the resulting frame
            cv.imshow('frame', frame)

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

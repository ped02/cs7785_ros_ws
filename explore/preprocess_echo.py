import argparse
import logging

import yaml

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


def main(input_file: str, output_file: str):
    logger.info(f"Processing '{input_file}'")

    data_array = []

    records_count = 0

    with open(input_file, 'r') as f:
        data = yaml.safe_load_all(f)
        for d in data:
            if d is not None:
                data_array.append(d)
                records_count += 1

    logger.info(f'Found {records_count} records')

    logger.info(f"Saving to '{output_file}'")
    with open(output_file, 'w') as f:
        yaml.safe_dump(data_array, f)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Preprocess ros2 topic echo to yaml format'
    )
    parser.add_argument(
        'input_file',
        type=str,
        help='Path to input yaml file that contains topic echo pipe.',
    )
    parser.add_argument(
        'output_file', type=str, help='Path to output yaml file'
    )

    args = parser.parse_args()

    setup_logger()

    # input_file = 'data/data.yml'
    # output_file = 'data/laser_scan_2.yml'

    main(args.input_file, args.output_file)

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wachakorn_navigate_to_goal'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rachanon',
    maintainer_email='ped02@me.com',
    description='TODO: Package description',
    license='Apache2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_to_point_cloud = wachakorn_navigate_to_goal.scan_to_point_cloud:main',
            'scan_point_cloud_window = wachakorn_navigate_to_goal.scan_point_cloud_window:main',
            'scan_segmenter = wachakorn_navigate_to_goal.scan_segmenter:main',
            'too_close_monitor = wachakorn_navigate_to_goal.too_close_monitor:main',
            'localizer = wachakorn_navigate_to_goal.localizer:main',
            'mapper = wachakorn_navigate_to_goal.mapper:main',
            'path_planner = wachakorn_navigate_to_goal.path_planner:main',
            'path_plan_tester = wachakorn_navigate_to_goal.path_plan_tester:main',
            'path_controller = wachakorn_navigate_to_goal.path_controller:main',
            'target_caller = wachakorn_navigate_to_goal.target_caller:main',
        ],
    },
)

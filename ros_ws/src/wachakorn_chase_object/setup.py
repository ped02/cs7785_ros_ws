from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wachakorn_chase_object'

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
        # ('share/' + package_name + 'launch', glob('launch/*launch.[pxy][yma]*'))
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
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_viewer = wachakorn_chase_object.image_viewer:main',
            'track_color = wachakorn_chase_object.track_color:main',
            'track_color_headless = wachakorn_chase_object.track_color_headless:main',
            'track_color_ui = wachakorn_chase_object.track_color_ui:main',
            'rotate_robot_bang_bang = wachakorn_chase_object.rotate_robot_bang_bang:main',
        ],
    },
)

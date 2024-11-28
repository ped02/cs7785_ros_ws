from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'wachakorn_nav_stack_command'

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
        (
            os.path.join('share', package_name, 'param'),
            glob(os.path.join('param', '*')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rachanon',
    maintainer_email='ped02@me.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'post_goal_real = wachakorn_nav_stack_command.post_goal_real:main',
            'post_goal_sim = wachakorn_nav_stack_command.post_goal_sim:main',
        ],
    },
)

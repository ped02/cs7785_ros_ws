from setuptools import find_packages, setup

package_name = 'dummy_data'

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
            'polygon_array_publisher = dummy_data.dummy_polygon_array_publisher:main',
            'polygon_publisher = dummy_data.dummy_polygon_stamped_publisher:main',
            'laser_scan_publisher = dummy_data.dummy_laser_scan_publisher:main',
        ],
    },
)

from setuptools import setup
import os
from glob import glob

package_name = 'robot_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 package for robot sensors',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node= robot_sensors.sensor_node:main',
            'stamp_fixer = robot_sensors.stamp_fixer:main',
            'ultrasonic_to_pointcloud = robot_sensors.ultrasonic_to_pointcloud:main',
        ],
    }
)

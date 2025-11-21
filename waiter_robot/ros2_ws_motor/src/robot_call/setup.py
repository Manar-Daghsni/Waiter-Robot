from setuptools import setup

package_name = 'robot_call'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adminrobot',
    maintainer_email='you@example.com',
    description='Robot call system with MQTT and BLE',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqtt_bridge = robot_call.mqtt_bridge:main',
            'call_manager = robot_call.call_manager:main',
            'ble_scanner = robot_call.ble_scanner:main',
        ],
    },
)

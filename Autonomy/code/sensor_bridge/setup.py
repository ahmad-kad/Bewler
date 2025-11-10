from setuptools import setup
import os
from glob import glob

package_name = 'autonomy_sensor_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='URC 2026 Team',
    maintainer_email='team@urc2026.com',
    description='WebSocket-to-ROS2 sensor data bridge for URC 2026 Mars Rover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_bridge_node = autonomy_sensor_bridge.websocket_bridge:main',
        ],
    },
)

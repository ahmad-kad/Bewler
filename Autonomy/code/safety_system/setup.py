from setuptools import setup
import os
from glob import glob

package_name = 'autonomy_safety_system'

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
        (os.path.join('share', package_name, 'resource'), ['resource/autonomy_safety_system']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='URC 2026 Safety Team',
    maintainer_email='safety@urc2026.com',
    description='Comprehensive safety system for URC 2026 Mars Rover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_watchdog = autonomy_safety_system.safety_watchdog:main',
            'redundant_safety_monitor = autonomy_safety_system.redundant_safety_monitor:main',
            'emergency_response_coordinator = autonomy_safety_system.emergency_response_coordinator:main',
            'safety_dashboard = autonomy_safety_system.safety_dashboard:main',
            'safety_integration_tester = autonomy_safety_system.safety_integration_tester:main',
        ],
    },
)

from setuptools import setup
import os
from glob import glob

package_name = 'autonomy_state_management'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'structlog', 'transitions'],
    zip_safe=True,
    maintainer='URC 2026 Team',
    maintainer_email='team@urc2026.edu',
    description='Hierarchical state machine for URC 2026 rover autonomy',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_machine_director = autonomy_state_management.state_machine_director:main',
        ],
    },
)
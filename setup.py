from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'grbl_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=[
        'pyserial',
    ],
    zip_safe=True,
    maintainer='Evan Flynn',
    maintainer_email='evanflynn.msu@gmail.com',
    description='ROS2 package to interface with a GRBL serial device',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grbl_node = grbl_ros.device:main',
            'grbl_service = grbl_ros.service:main',
            'grbl_manager = grbl_ros.manager:main'
        ],
    },
)

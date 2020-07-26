from glob import glob
import os

from setuptools import setup

package_name = 'grbl_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Evan Flynn',
    maintainer_email='evanflynn.msu@gmail.com',
    description='ROS2 package to interface with a GRBL serial device',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interface = grbl_ros.interface:main'
        ],
    },
)

from setuptools import setup
import os
from glob import glob

package_name = 'aruco_realsense_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('aruco_realsense_ros/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosdev',
    maintainer_email='rosdev@todo.todo',
    description='ROS2 node for ArUco marker detection using RealSense camera',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_node = aruco_realsense_ros.aruco_node:main',
        ],
    },
)

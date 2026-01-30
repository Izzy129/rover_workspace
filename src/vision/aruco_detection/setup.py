from setuptools import setup
import os
from glob import glob

package_name = 'aruco_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=[
        'setuptools',
        'opencv-contrib-python>=4.8.0,<4.9.0',
        'numpy>=1.21.2,<2.0',
    ],
    zip_safe=True,
    maintainer='Nathan Sprague',
    maintainer_email='nathan.r.sprague@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_node = aruco_detection.aruco_node:main',
            'aruco_generate_marker = aruco_detection.aruco_generate_marker:main',
            'webcam_publisher = aruco_detection.webcam_publisher:main',
            'realsense_publisher = aruco_detection.realsense_publisher:main',
            'keyboard_detection = aruco_detection.keyboard_detection:main',
            'keyboard_visualizer = aruco_detection.keyboard_visualizer:main',
        ],
    },
)

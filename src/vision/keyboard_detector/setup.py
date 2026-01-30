from setuptools import setup
import os
from glob import glob

package_name = 'keyboard_detector'

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
    maintainer='R-owl-vers',
    maintainer_email='riceroboticsclub@gmail.com',
    description='Keyboard pose estimation from ArUco markers for URC',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_detection = keyboard_detector.keyboard_detection:main',
            'keyboard_visualizer = keyboard_detector.keyboard_visualizer:main',
        ],
    },
)

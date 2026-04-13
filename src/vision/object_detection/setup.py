from setuptools import setup

package_name = 'object_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['object_detection/best.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='riceroboticsclub@gmail.com',
    description='Object detection',
    license='TODO',
    entry_points={
        'console_scripts': [
            'detector = object_detection.detector:main',
        ],
    },
)

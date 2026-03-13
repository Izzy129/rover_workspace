from setuptools import find_packages, setup

package_name = 'genesis_arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/joint_limits.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ericg4',
    maintainer_email='eric.g4@outlook.com',
    description='Arm controller with joint limits',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arm_controller = genesis_arm_control.controller_node:main',
        ],
    },
)

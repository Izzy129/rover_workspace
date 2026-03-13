from setuptools import find_packages, setup

package_name = 'genesis_arm_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    scripts=['scripts/keyboard_teleop_venv'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ericg4',
    maintainer_email='eric.g4@outlook.com',
    description='Keyboard teleop for Genesis arm',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'keyboard_teleop = genesis_arm_teleop.keyboard_node:main',
        ],
    },
)

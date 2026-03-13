from setuptools import find_packages, setup

package_name = 'genesis_arm_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    scripts=['scripts/genesis_bridge_venv'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ericg4',
    maintainer_email='eric.g4@outlook.com',
    description='Genesis simulation bridge for arm',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'genesis_bridge = genesis_arm_bridge.bridge_node:main',
        ],
    },
)

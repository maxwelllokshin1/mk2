from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mcu_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='maxwell.lokshin@2zick.com',
    description='Bridge from /drive to a serial MCU or Pi GPIO',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'bridge_node = mcu_bridge.bridge_node:main',
            'steering_sweep = mcu_bridge.steering_sweep:main',
            'serial_steer_test = mcu_bridge.serial_steer_test:main',
        ],
    },
)

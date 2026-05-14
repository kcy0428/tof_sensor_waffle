from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cliff_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='PC-side cliff detection monitor for TurtleBot3 ToF sensor',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cliff_monitor_node = cliff_monitor.cliff_monitor_node:main',
            'tof_viewer = cliff_monitor.tof_viewer:main',
        ],
    },
)

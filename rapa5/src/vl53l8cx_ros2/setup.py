from setuptools import find_packages, setup

package_name = 'vl53l8cx_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='VL53L8CX 8x8 ToF sensor ROS2 driver',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tof_serial_node = vl53l8cx_ros2.tof_serial_node:main',
            'cliff_detector_node = vl53l8cx_ros2.cliff_detector_node:main',
        ],
    },
)

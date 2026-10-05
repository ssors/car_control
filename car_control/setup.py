from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'car_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='susu',
    maintainer_email='susu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'smooth_teleop = car_control.smooth_teleop:main',
            'auto_explorer = car_control.auto_explorer:main',
            'check_lidar = car_control.check_lidar:main',
            'cruise_node = car_control.cruise_node:main',
            
            
            'stop_car = car_control.stop_car:main',
            'coverage_planner = car_control.coverage_planner:main',
            'coverage_path_generator = car_control.coverage_path_generator:main',
            'coverage_navigator = car_control.coverage_navigator:main',
            # 'door_margin_scan = car_control.door_margin_scan:main',
            'manual_room_segment = car_control.manual_room_segment:main',
            'unstuck_helper = car_control.unstuck_helper:main',
            'serial_bridge = car_control.serial_bridge:main',
            
        ],
    },
)

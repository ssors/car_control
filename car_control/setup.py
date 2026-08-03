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
            
            'stop_car = car_control.stop_car:main',
            'odom_monitor = car_control.odom_monitor:main',
        ],
    },
)

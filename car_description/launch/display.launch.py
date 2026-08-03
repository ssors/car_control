"""
在 RViz 中查看小车模型（不启动 Gazebo）。
用法：
    ros2 launch car_description display.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('car_description')

    xacro_file = os.path.join(pkg_share, 'urdf', 'car.urdf.xacro')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'car.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_gazebo:=false']),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': use_sim_time}],
        ),

        # 没有真实/仿真硬件反馈关节角度时，用 GUI 滑条手动模拟各关节角度
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
        ),
    ])

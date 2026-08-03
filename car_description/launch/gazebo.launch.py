"""
在 Gazebo 中加载小车模型（Gazebo Classic + gazebo_ros_pkgs）。
用法：
    ros2 launch car_description gazebo.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('car_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    world_path = os.path.join(pkg_share, 'world', 'my_world.world')

    xacro_file = os.path.join(pkg_share, 'urdf', 'car.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')
    pause = LaunchConfiguration('pause')
    use_ackermann_plugin = LaunchConfiguration('use_ackermann_plugin')

    # 关掉 Gazebo 联网访问 Fuel 在线模型库的行为。
    # WSL2 里 DNS 经常连不上 fuel.ignitionrobotics.org，反复超时重试
    # 会明显拖慢 gzserver 启动速度，导致 /spawn_entity 服务迟迟没准备好。
    set_model_db_empty = SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', '')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' use_gazebo:=true',
                 ' use_ackermann_plugin:=', use_ackermann_plugin]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        # pause:=true 让世界一开始就是暂停状态，方便用 gzclient 里的
        # "Step" 按钮一帧一帧看模型是否从一开始就是对的
        launch_arguments={'verbose': 'false', 'pause': 'false', 'world': world_path}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}],
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_ackermann_car',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'ackermann_car',
                   '-z', '0.05', '-timeout', '120.0'],
    )

    # WSL2 下 gzserver 启动经常比较慢，延迟 8 秒再发生成请求，
    # 避免 Gazebo 服务还没就绪、spawn 请求就先超时退出。
    spawn_entity = TimerAction(period=8.0, actions=[spawn_entity_node])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'pause', default_value='true',
            description='Start Gazebo paused so the initial pose can be inspected frame by frame'),
        DeclareLaunchArgument(
            'use_ackermann_plugin', default_value='true',
            description='Set to false to disable the ackermann drive plugin for debugging'),
        set_model_db_empty,
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 1. 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    map_yaml_file = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    # 【注意修此处的路径】默认路径配置
    default_map_path = os.path.expanduser('~/ros2/ros2_ws/src/ros2_maps/room_latest.yaml')
    default_nav2_params = os.path.expanduser('~/ros2/ros2_ws/src/my_nav_config/raw_nav2_params.yaml')
    default_rviz_config = os.path.expanduser('~/ros2/ros2_ws/src/car_description/config/nav_default.rviz')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='true')
    declare_gazebo_gui = DeclareLaunchArgument('gazebo_gui', default_value='false')
    declare_map = DeclareLaunchArgument('map', default_value=default_map_path, description='已保存的地图yaml路径')
    declare_nav2_params_file = DeclareLaunchArgument('nav2_params_file', default_value=default_nav2_params)
    declare_rviz_config_file = DeclareLaunchArgument('rviz_config_file', default_value=default_rviz_config)
    # ekf_params_file ='/home/susu/ros2/ros2_ws/src/my_nav_config/raw_nav2_ekf.yaml'
    
    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[ekf_params_file, {'use_sim_time': use_sim_time}]
    # )


    # 2. 启动 Gazebo 仿真环境 (先启动)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('car_description'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'gui': gazebo_gui}.items()
    )

    # 3. 启动 Nav2 Bringup (延时 5 秒启动，等 Gazebo 模型生成完并发布 odom)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': nav2_params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'true', # 确保生命周期节点自动激活
        }.items()
    )

    delayed_nav2_launch = TimerAction(
        period=5.0,  # 延迟 5 秒
        actions=[nav2_bringup_launch]
    )

    # 4. RViz2 可视化 (同样延时 5 秒)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    delayed_rviz_node = TimerAction(
        period=5.0,
        actions=[rviz_node]
    )
    
    DELAY = 8.0
    delayed_nav2 = TimerAction(period=DELAY, actions=[nav2_bringup_launch])
    delayed_rviz = TimerAction(period=DELAY, actions=[rviz_node])
    # delayed_ekf = TimerAction(period=DELAY, actions=[ekf_node])


    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        declare_gazebo_gui,
        declare_map,
        declare_nav2_params_file,
        declare_rviz_config_file,

        gazebo_launch,
        
        
        
        delayed_nav2,
        delayed_rviz,
    ])
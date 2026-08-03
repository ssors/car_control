"""
自主建图一键启动 launch 文件

等价于依次手动执行以下命令：
  ros2 launch car_description gazebo.launch.py gui:=false
  ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
  ros2 launch nav2_bringup navigation_launch.py params_file:=... use_sim_time:=true
  ros2 launch explore_lite explore.launch.py use_sim_time:=true
  ros2 run rviz2 rviz2 -d ... --ros-args -p use_sim_time:=true

用法：
  ros2 launch <你的包名> mapping_launch.py

注意：
  1. 把本文件放到你自己的功能包（例如 my_nav_config 或 car_description）的
     launch/ 目录下，并在 package.xml / CMakeLists.txt（或 setup.py，若是 Python 包）
     里确保 launch 目录会被安装。
  2. 下面几个路径变量按你自己的实际路径检查一遍，尤其是 params_file 和 rviz 配置文件。
  3. 保存地图（map_saver_cli）不适合放进这个 launch 里长期跑着，
     建图结束后手动执行一次即可：
       ros2 run nav2_map_server map_saver_cli -f ~/ros2/ros2_ws/src/maps/map_v1
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ------------------------------------------------------------------
    # 可以在命令行里覆盖的参数，例如：
    #   ros2 launch <包名> mapping_launch.py use_rviz:=false
    # ------------------------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    
        # 1. 动态获取路径
    map_dir = os.path.expanduser('~/ros2/ros2_ws/src/maps')
    mask_yaml_file = os.path.join(map_dir, 'keepout_mask.yaml')

    my_nav_config_dir = os.path.expanduser('~/ros2/ros2_ws/src/my_nav_config')
    keepout_params_file = os.path.join(my_nav_config_dir, 'keepout_params.yaml')

    # 2. 定义 Keepout Zone 专属的三个核心节点
    filter_mask_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        emulate_tty=True,
        parameters=[
            keepout_params_file,
            {'yaml_filename': mask_yaml_file}
        ]
    )

    costmap_filter_info_server_node = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        emulate_tty=True,
        parameters=[keepout_params_file]
    )

    lifecycle_manager_costmap_filters_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['filter_mask_server', 'costmap_filter_info_server']
        }]
    )

    # 3. 别忘了将这三个节点放入最后的 LaunchDescription([]) 列表中！

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='是否启动 rviz2'
    )

    declare_gazebo_gui = DeclareLaunchArgument(
        'gazebo_gui',
        default_value='false',
        description='是否显示 Gazebo 图形界面'
    )

    declare_nav2_params_file = DeclareLaunchArgument(
        'nav2_params_file',
        default_value='/home/susu/ros2/ros2_ws/src/my_nav_config/my_nav2_params.yaml',
        description='nav2 参数文件路径'
    )

    declare_rviz_config_file = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            os.path.expanduser('~'),
            'ros2/ros2_ws/src/car_description/config/default.rviz'
        ),
        description='rviz2 配置文件路径'
    )

    # ------------------------------------------------------------------
    # 1. Gazebo 仿真
    # ------------------------------------------------------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('car_description'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'gui': gazebo_gui,
        }.items()
    )

    # ------------------------------------------------------------------
    # 2. slam_toolbox 建图
    # ------------------------------------------------------------------
    slam_toolbox_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('slam_toolbox'),
            'launch',
            'online_async_launch.py'
        )
    ),
    launch_arguments={
        'use_sim_time': use_sim_time,
        # 'slam_params_file': '/home/susu/ros2/ros2_ws/src/my_nav_config/my_slam_params.yaml',
    }.items()
)

    # ------------------------------------------------------------------
    # 3. nav2 导航（controller/planner/costmap/behavior 等）
    # ------------------------------------------------------------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # ------------------------------------------------------------------
    # 4. explore_lite 自主探索
    # ------------------------------------------------------------------
    explore_lite_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('explore_lite'),
                'launch',
                'explore.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # ------------------------------------------------------------------
    # 5. rviz2 可视化（可选）
    # ------------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        declare_gazebo_gui,
        declare_nav2_params_file,
        declare_rviz_config_file,

        gazebo_launch,
        slam_toolbox_launch,
        nav2_launch,
        explore_lite_launch,
        rviz_node,
        
        filter_mask_server_node,
        costmap_filter_info_server_node,
        lifecycle_manager_costmap_filters_node,
    ])
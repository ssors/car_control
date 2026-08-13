
```markdown
# ROS 2 Autonomous Car Control & Navigation Workspace

本项目是一个基于 **ROS 2** 的移动机器人/小车控制与自主导航工作区，包含机器人模型描述、底盘控制、建图、自主探索（m-explore）以及 Nav2 导航配置等完整功能模块。

---

## 目录结构说明

```text
src/
├── car_control/        # 机器人底层运动控制与建图 Launch 脚本
│   ├── car_control/    # 节点源码 (雷达检查、全屋巡检、路径规划、地图分割房间、键盘控制、车辆停止和测试代码)
│   ├── coverage_out/   # 分房间和路径规划输出  
│   ├── launch/         # 启动文件 (建图和导航)
│   ├── resource/       # 资源文件
│   └── test/           # 自动化代码测试脚本
├── car_description/    # 机器人 URDF/Xacro 3D 模型与 Gazebo 仿真环境定义
│   ├── config/         # Rviz/Gazebo 相关配置
│   ├── launch/         # 仿真启动脚本 (display.launch.py / gazebo.launch.py)
│   ├── urdf/           # 机器人部件描述文件 (底盘、轮子、雷达、相机)
│   └── world/          # Gazebo 仿真地图场景文件 (*.world)
├── m-explore-ros2/     # ROS 2 自主边界探索算法库 (Frontier Exploration)
│   ├── explore/        # 自主探索主功能包
│   └── explore_lite_msgs/ # 自主探索自定义消息接口
├── ros2_maps/          # 建图保存文件（room_latest是最新）
└── my_nav_config/      # Nav2 导航与 SLAM建图参数配置文件 (*.yaml / *.xml)


```

---

## 主要功能包介绍

### 1. `car_control` (控制、建图和导航)

* **功能**：提供小车平滑遥控（`smooth_teleop.py`）、紧急制动（`stop_car.py`）、雷达状态检测（`check_lidar.py`）、全屋巡检路径规划（`coverage_navigator.py`、`coverage_path_generator.py`、`manual_room_segment.py`）以及一键启动 SLAM 建图和导航视角的 Launch 脚本。
* **主要文件**：
* `launch/mapping.launch.py`: 结合雷达与 SLAM 的建图启动文件。
* `launch/navigation.launch.py`: 包括可视化rviz和gazebo的导航启动文件。



### 2. `car_description` (机器人模型与仿真)

* **功能**：包含小车底盘、轮子、激光雷达（Lidar）和摄像头（Camera）的 URDF/Xacro 机械建模，以及 Gazebo 仿真世界环境（如 `room.world` / `my_world.world`）。

### 3. `m-explore-ros2` (自主探索)

* **功能**：集成基于 Frontier 边界的自主建图算法，无需手动遥控，小车即可自动探索未知区域并建立地图。

### 4. `my_nav_config` (导航与 SLAM 配置)

* **功能**：针对该小车量身定制的 Nav2 导航参数与行为树配置文件。
* **主要文件**：
* `my_nav2_params.yaml`: Nav2 代价地图与路径规划器参数。（导航建图用）
* `my_navigation_to_pose.yaml`: bt行为树参数。（导航建图用）
* `raw_nav2_params.yaml`: Nav2 代价地图与路径规划器参数。（导航巡检用）
* `raw_nav2_to_pose.yaml`: bt行为树参数。（导航巡检用）
* `my_slam_params.yaml`: SLAM 建图参数。



### 5. `ros2_maps` (地图资源)

* 存放建图完成的静态地图文件以及对应的代价地图掩码。

### 6. `coverage_output`（全屋覆盖路线可视化）

* `manual_room_split.png` : 地图分房间预览图
* `coverage_preview.png`  : 路线规划预览图

---

## 🛠️ 环境依赖与编译指南

### 1. 环境准备

确保已安装 **ROS 2** (如 Humble / Iron 等) 及相关依赖包：

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-nav2-bringup \
                 ros-${ROS_DISTRO}-navigation2 \
                 ros-${ROS_DISTRO}-slam-toolbox \
                 ros-${ROS_DISTRO}-gazebo-ros-pkgs \
                 ros-${ROS_DISTRO}-gazebo-ros2-control \
                 ros-${ROS_DISTRO}-xacro \
                 ros-${ROS_DISTRO}-robot-state-publisher \
                 ros-${ROS_DISTRO}-joint-state-publisher \
                 ros-${ROS_DISTRO}-joint-state-publisher-gui \
                 ros-${ROS_DISTRO}-rviz2

pip install opencv-python numpy scipy matplotlib pyyaml --break-system-packages


```

### 2. 编译工作区

回到工作区根目录（`ros2_ws`）：

```bash
cd ~/ros2/ros2_ws
colcon build 
source install/setup.bash

```

---

## 🎮 使用说明

### 1. 启动建图 (SLAM)(直接启动这一个就可以，设置的是gazebo无图模式，只显示rviz2)

```bash
ros2 launch car_control mapping.launch

```
### 2. 启动全屋巡检 (设置的是gazebo无图模式，只显示rviz2)

```bash
ros2 launch car_control navigation.launch 
ros2 run car_control coverage_navigator 

```
### 3.启动全屋巡检前准备（分房间、路径规划）
```bash
ros2 run car_control manual_room_segment
ros2 run car_control coverage_path_generator

```
#### 提醒：使用完manual_room_segment后手动关闭图形窗口，不要用Ctrl+C退出，不然不会更新保存。
---

## 📝 License

[MIT License / Apache-2.0]

```

---

### 保存并提交到 GitHub

在 VS Code 终端里顺序敲这三行命令即可更新：

```bash
git add README.md
git commit -m "添加完整版 README 项目说明"
git push

```

```markdown
# ROS 2 Autonomous Car Control & Navigation Workspace

本项目是一个基于 **ROS 2** 的移动机器人/小车控制与自主导航工作区，包含机器人模型描述、底盘控制、建图、自主探索（m-explore）以及 Nav2 导航配置等完整功能模块。

---

## 📁 目录结构说明

```text
src/
├── car_control/        # 机器人底层运动控制与建图 Launch 脚本
│   ├── car_control/    # 节点源码 (控制脚本、雷达检查、自动探索辅助等)
│   ├── launch/         # 启动文件 (建图 mapping.launch.py 等)
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
├── maps/               # 保存的建图成果 (.pgm/.yaml) 与禁行区掩码 (keepout mask)
└── my_nav_config/      # Nav2 导航与 SLAM建图参数配置文件 (*.yaml / *.xml)

```

---

## 🚀 主要功能包介绍

### 1. `car_control` (控制与建图)

* **功能**：提供小车平滑遥控（`smooth_teleop.py`）、紧急制动（`stop_car.py`）、雷达状态检测（`check_lidar.py`）以及一键启动 SLAM 建图的 Launch 脚本。
* **主要文件**：
* `launch/mapping.launch.py`: 结合雷达与 SLAM 的建图启动文件。



### 2. `car_description` (机器人模型与仿真)

* **功能**：包含小车底盘、轮子、激光雷达（Lidar）和摄像头（Camera）的 URDF/Xacro 机械建模，以及 Gazebo 仿真世界环境（如 `room.world` / `my_world.world`）。

### 3. `m-explore-ros2` (自主探索)

* **功能**：集成基于 Frontier 边界的自主建图算法，无需手动遥控，小车即可自动探索未知区域并建立地图。

### 4. `my_nav_config` (导航与 SLAM 配置)

* **功能**：针对该小车量身定制的 Nav2 导航参数与行为树配置文件。
* **主要文件**：
* `my_nav2_params.yaml`: Nav2 代价地图与路径规划器参数。
* `keepout_params.yaml`: 禁行区掩码配置。
* `my_slam_params.yaml`: SLAM 建图参数。



### 5. `maps` (地图资源)

* 存放建图完成的静态地图文件（`map_v1.pgm/.yaml`）以及对应的代价地图掩码（`keepout_mask.pgm/.yaml`）。

---

## 🛠️ 环境依赖与编译指南

### 1. 环境准备

确保已安装 **ROS 2** (如 Humble / Iron 等) 及相关依赖包：

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-nav2-bringup \
                 ros-${ROS_DISTRO}-navigation2 \
                 ros-${ROS_DISTRO}-slam-toolbox

```

### 2. 编译工作区

回到工作区根目录（`ros2_ws`）：

```bash
cd ~/ros2/ros2_ws
rosdep install -i --from-paths src --rosdistro ${ROS_DISTRO} -y
colcon build --symlink-install
source install/setup.bash

```

---

## 🎮 使用说明

### 1. 启动建图 (SLAM)(直接启动这一个就可以，设置的是gazebo无图模式，只显示rviz2)

```bash
ros2 launch car_control mapping.launch.py 

```



```

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
# car_description —— 阿克曼四轮小车 URDF/Xacro 模型

## 1. 目录结构

```
car_description/
├── package.xml
├── CMakeLists.txt
├── urdf/
│   ├── car.urdf.xacro          # 主入口，把下面所有文件 include 在一起
│   ├── common_properties.xacro # 所有尺寸/质量/摩擦参数（改这里即可整体调整）
│   ├── materials.xacro         # RViz 显示颜色
│   ├── inertial_macros.xacro   # 惯量自动计算宏（长方体/圆柱体）
│   ├── base.xacro              # base_footprint + 下层底盘 + 支柱 + 上层底盘
│   ├── wheel.xacro             # 前轮(转向节+车轮) / 后轮(固定轴车轮) 宏 + 实例化4个轮子
│   ├── lidar.xacro             # N10 雷达 link + Gazebo 2D激光插件
│   └── car.gazebo.xacro        # Gazebo 材质 + 阿克曼驱动插件
├── launch/
│   ├── display.launch.py       # 纯 RViz 查看（joint_state_publisher_gui 手动拖关节）
│   └── gazebo.launch.py        # 加载进 Gazebo，可用 /cmd_vel 控制
└── rviz/car.rviz
```

## 2. 关节/坐标树 (TF Tree)

```
base_footprint (地面, z=0)
└── base_link  (下层底盘几何中心, fixed)
    ├── upper_plate_link (上层底盘, fixed, 经支柱抬高)
    │   └── lidar_link (fixed, 上层前1/4正中)
    ├── standoff_fl/fr/rl/rr_link (4根支柱, fixed, 纯结构可视化)
    ├── steer_left_link  --(revolute, 绕Z)--> front_left_wheel_link  --(continuous, 绕Y)
    ├── steer_right_link --(revolute, 绕Z)--> front_right_wheel_link --(continuous, 绕Y)
    ├── rear_left_wheel_link  (continuous, 绕Y, 固定轴)
    └── rear_right_wheel_link (continuous, 绕Y, 固定轴)
```

阿克曼转向的核心是：前轮 = "转向节(steer_link)" 先绕竖直Z轴转，再在转向节上挂"车轮(wheel_link)"绕Y轴自转；后轮直接绕Y轴自转、不转向。

## 3. 题目给定参数（已直接采用）

| 参数 | 数值 |
|---|---|
| 总质量 | 1.8 kg |
| 车总高 | 70 mm = 离地间隙20 + 下层板厚15 + 支柱20 + 上层板厚15 (mm) |
| 车长 | 220 mm |
| 前轮距车头 | 20 mm（前轴 x = 车长/2 − 20mm = 90mm）|
| 后轮与车尾平齐 | 后轴 x = −车长/2 = −110mm，轴距 = 200mm |
| 下层底盘离地 | 20 mm |
| 雷达型号/尺寸/质量 | N10，Φ52×36.1mm，80g，位于上层前1/4正中 |
| 车轮宽度/直径 | 25 mm / 65 mm（半径32.5mm）|

## 4. 题目未给出、由我补充的假设值（在 `common_properties.xacro` 中均有中文注释标注"【假设值】"，可直接修改）

- **车宽**：0.15 m
- **左右轮距（轮中心间距）**：0.185 m
- **下层板厚 / 支柱高 / 上层板厚**：15 / 20 / 15 mm（三者之和 + 离地间隙 正好凑成给定的总高70mm）
- **单个车轮质量**：0.1 kg（4个共0.4kg）
- **转向节(steer_link)质量**：0.03 kg（2个共0.06kg，比几何本身该有的质量明显偏大，是为了避免物理仿真数值不稳定，见第7节）
- **上/下层底盘质量分配**：0.68 kg / 0.56 kg（4根支柱各5g），使总质量精确等于1.8kg
- **最大转向角**：±30°（0.5236 rad），阿克曼小车常见取值
- **关节阻尼/摩擦、轮胎与地面摩擦系数(mu1/mu2/kp/kd)**：给出了橡胶轮胎典型的 Gazebo ODE 参数（mu1=mu2=1.0），可按实测调整
- **N10雷达的扫描参数**（角分辨率/量程/频率）：按常见2D雷达给了典型值，请对照 N10 实际规格书调整 `lidar.xacro` 里 `<ray>` 部分

> 说明：车轮半径(32.5mm) 略大于下层底盘离地间隙(20mm)，因此车轮轴心比下层底盘下表面高出12.5mm——这是很多小车的常见结构（轮子从底盘侧面探出），符合题目给出的两个数值，无需强行改动。

## 5. 惯量如何计算的

`inertial_macros.xacro` 提供三个宏，均按**均匀密度刚体**理论公式自动计算，不需要手填惯量矩阵：

- `inertial_box`：长方体（下层板/上层板）
  - Ixx = m(y²+z²)/12, Iyy = m(x²+z²)/12, Izz = m(x²+y²)/12
- `inertial_cylinder_z`：轴沿Z（雷达、支柱）
  - Ixx=Iyy = m(3r²+h²)/12, Izz = m·r²/2
- `inertial_cylinder_y`：轴沿Y（车轮，因为车轮绕Y轴滚动）
  - Ixx=Izz = m(3r²+h²)/12, Iyy = m·r²/2

改了尺寸/质量后惯量会自动重新算，不用手动改惯量矩阵。

## 6. 在 WSL2 中如何使用

假设你已经在 WSL2 里装好了 ROS2（本文以 Gazebo Classic + gazebo_ros_pkgs 为例，对应 Humble 默认自带的 Gazebo 11；如果你用的是新版 Gazebo(Harmonic)+ros_gz，请看第7节说明）。

```bash
# 1. 放进工作空间
mkdir -p ~/car_ws/src
cp -r car_description ~/car_ws/src/
cd ~/car_ws

# 2. 安装依赖（如未装）
sudo apt update
sudo apt install ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-joint-state-publisher-gui \
                  ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-gazebo-ros-pkgs \
                  ros-$ROS_DISTRO-gazebo-ros

# 3. 编译
colcon build --packages-select car_description
source install/setup.bash

# 4a. 只看 RViz（用滑条手动转关节，检查模型/关节连接是否正确）
ros2 launch car_description display.launch.py

# 4b. 进 Gazebo 仿真（WSL2 需要先装好 WSLg 或 VcXsrv 等能显示 GUI）
ros2 launch car_description gazebo.launch.py

# 在另一个终端用键盘控制阿克曼小车（前进后退+转向）
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

WSL2 小提示：
- Ubuntu 22.04(WSL2) 自带 WSLg，一般不用额外配置就能弹出 Gazebo/RViz 窗口；如果打不开窗口，检查 `echo $DISPLAY` 是否有输出，或安装 VcXsrv 并 `export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0`。
- 若 Gazebo 打开很慢/卡，是 WSL2 下 OpenGL 软件渲染的通病，可以先只用 `display.launch.py` 在 RViz 里验证模型结构正确，再进 Gazebo。

## 7. 校验模型是否正确（强烈建议做）

```bash
xacro car_description/urdf/car.urdf.xacro > /tmp/car.urdf
check_urdf /tmp/car.urdf          # 检查关节树/父子link是否都连上了，有没有孤立link
urdf_to_graphiz /tmp/car.urdf     # 生成 car.pdf，可视化查看TF树结构是否符合预期
```

## 8. 如果你用的是新版 Gazebo (Harmonic/Fortress, ros_gz)

`car.gazebo.xacro` 里的 `libgazebo_ros_ackermann_drive.so` 插件是 **Gazebo Classic** 专用的，新版 Gazebo 不能直接用。需要把该 `<plugin>` 换成 `gz_ros2_control` 的 `<ros2_control>` 标签 + `ackermann_steering_controller` 的 controller yaml 配置（结构类似，但语法不同）。如果你确认自己用的是新版 Gazebo，告诉我一声，我可以把这部分单独改写。

## 9. 摄像头（杆装 USB 免驱摄像头）

新增 `urdf/camera.xacro`，装在上层底盘正中间，结构是：

```
upper_plate_link
  └─ pole_link (支撑杆, Φ8mm×200mm, 竖直)
      └─ camera_base_link (安装底座, 38×38×5mm)
          └─ camera_link (摄像头本体, Φ14×22mm镜头筒)
              └─ camera_link_optical (仅做坐标系转换，ROS图像话题用这个frame_id)
```

- **朝向/俯仰角**：摄像头圆柱本身在自己的坐标系里是**竖直的**（垂直于底座，符合真实产品的机械结构），朝向车头方向完全靠 `camera_joint` 这一个关节的倾斜角度实现——`camera_tilt_angle = camera_base_tilt(90°，把竖直转到水平朝前) + camera_extra_downtilt(15°，再往下看一点)`，整体朝 +X（雷达/车头那一侧）倾斜。想调"往下看多少"只改 `camera_extra_downtilt` 这一个属性即可，90°那部分是几何关系决定的、不要动。
- **质量**：杆+底座+摄像头本体共新增约50g，是在原来1.8kg基础上**额外增加**的（如果要严格保持总重1.8kg不变，需要相应减掉底盘的质量，我在xacro注释里标了这一点）。
- **Gazebo仿真**：装了 `libgazebo_ros_camera.so` 插件，仿真里会发布 `/camera/image_raw`（`sensor_msgs/Image`）和 `/camera/camera_info`，可以用 `rqt_image_view` 订阅 `/camera/image_raw` 查看画面。
- **光学坐标系说明**：`camera_link`本身跟车身惯例一致(X朝前)，但ROS图像处理惯例是Z朝前、X右、Y下，两者差一个固定旋转，单独用`camera_link_optical`这个虚拟frame承载，Gazebo插件的`frame_name`已经指向它，图像话题的frame_id会是`camera_link_optical`，这是标准做法，不用改。

## 10. 遇到过的一个坑：车轮在 Gazebo 里播放后"飞走/缩到中心"（后来发现主要是 gzclient 渲染问题，见下方补充）

现象：RViz 里模型正常、Gazebo 暂停时位置也正常，但一旦取消暂停开始跑物理，车轮位置就乱了（比如看起来只剩一个轮子在底盘中间）。

排查后确认根因是**车轮与地面的接触参数(kp)设得太"硬"**：车轮很轻（0.1kg），配合默认1ms物理步长，过高的接触刚度会让 ODE 解算器在修正微小穿透时用过大的力把车轮"弹飞"，同时伴随 `Real Time Factor` 骤降到接近0（解算器在拼命挣扎）。

修复：
- 把 `wheel_kp` 从 `1e6` 降到 `1e5`，`wheel_kd` 从 `1.0` 提到 `10.0`（更柔和的接触）
- 给车轮碰撞体加了 `<maxVel>` 和 `<minDepth>`，限制穿透修正的最大速度，防止"暴力纠正"
- 把转向节(`steer_link`)的质量从 0.01kg 提到 0.03kg——太轻的连杆夹在重得多的车轮和底盘之间，也是数值不稳定的常见诱因

如果你改了尺寸/质量后又复现类似问题，可以按这个思路查：**先看 Real Time Factor 是否明显低于1**，如果是，大概率是接触参数或者某个连杆质量太轻导致的数值不稳定，而不是坐标算错了（坐标算错在 RViz 里就能看出来，暂停状态位置就已经不对）。

**补充（重要）**：后续用 `tf2_echo` 直接查询物理引擎里各车轮相对 `base_footprint` 的实时坐标，确认在跑了1分钟以上之后数值依然完全正确、互不重叠。而 gzclient(Gazebo自带GUI窗口) 画面上却显示四个轮子挤在一起。这说明真正的问题是 **WSL2 下 Gazebo Classic 的 gzclient 渲染 bug**，不是模型/坐标/物理参数的问题——物理引擎(gzserver)内部状态从头到尾都是对的。如果你也遇到类似"画面挤在一起但TF显示位置正确"的情况，不用改模型，可以按顺序试：设置 `export LIBGL_ALWAYS_SOFTWARE=1`、换用 VcXsrv 代替默认WSLg、更新显卡驱动，或者干脆不依赖 gzclient 画面、改用 RViz 或命令行(`tf2_echo`/`ros2 topic echo /odom`)来验证仿真效果。

## 11. 关于结构合理性的建议

- 目前"支柱"只是4根细圆柱（纯占位/连接作用），实际打印/组装时你可能会用亚克力柱、螺柱等，把 `standoff_radius`、`mass_standoff` 改成你实际用的规格即可。
- 车宽(150mm)、轮距(185mm)是我按小车整体比例给的估计值，如果你有实际车宽/轮距数据，改 `common_properties.xacro` 里的 `chassis_width` 和 `wheel_y_offset` 两个属性即可，其余全部自动联动。

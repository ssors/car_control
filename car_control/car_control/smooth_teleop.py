"""
自定义平滑键盘控制节点 (ROS 2 - Python)
适用于阿克曼机器人，支持平滑加减速，防止 Gazebo 物理仿真冲击打滑
"""

import os
import select
import sys
import termios
import tty    # 用于非阻塞键盘输入

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# 按键绑定映射
KEY_BINDINGS = {
    'w': (1.0, 0.0),  # 前进
    's': (-1.0, 0.0),  # 后退
    'a': (0.0, 1.0),  # 左转 (角速度为正)
    'd': (0.0, -1.0),  # 右转 (角速度为负)
    'q': (1.0, 1.0),  # 前进 + 左转
    'e': (1.0, -1.0),  # 前进 + 右转
    'z': (-1.0, -1.0),  # 后退 + 左转
    'c': (-1.0, 1.0),  # 后退 + 右转
}

# 调速按键
SPEED_BINDINGS = {
    'i': (0.1, 0.0),  # 增加最大线速度
    'k': (-0.1, 0.0),  # 减少最大线速度
    'j': (0.0, 0.1),  # 增加最大角速度
    'l': (0.0, -0.1),  # 减少最大角速度
}


def get_key(settings):
    """获取单字符键盘输入（非阻塞模式）"""
    tty.setraw(sys.stdin.fileno())  # setraw用来把终端设置为原始模式，输入无需回车
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class SmoothTeleopNode(Node):

    def __init__(self):
        super().__init__('smooth_teleop')

        # 1. 声明并获取参数（可在 launch 文件中灵活修改）
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('max_linear_speed', 0.5)  # m/s
        self.declare_parameter('max_angular_speed', 0.0)  # rad/s
        self.declare_parameter('linear_accel', 0.4)  # m/s^2 线加速度
        self.declare_parameter('angular_accel', 1.0)  # rad/s^2 角加速度

        topic_name = (
            self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        )
        self.max_lin = (
            self.get_parameter('max_linear_speed').get_parameter_value().double_value
        )
        self.max_ang = (
            self.get_parameter('max_angular_speed').get_parameter_value().double_value
        )
        self.lin_accel = (
            self.get_parameter('linear_accel').get_parameter_value().double_value
        )
        self.ang_accel = (
            self.get_parameter('angular_accel').get_parameter_value().double_value
        )

        # 2. 创建发布者
        self.pub = self.create_publisher(Twist, topic_name, 10)

        # 3. 内部运动状态变量
        self.target_lin = 0.0  # 目标线速度
        self.target_ang = 0.0  # 目标角速度
        self.current_lin = 0.0  # 当前实际发布的线速度
        self.current_ang = 0.0  # 当前实际发布的角速度

        # 4. 定时器：20Hz (0.05s) 定时平滑计算并发布速度
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.get_logger().info(f'平滑键盘控制节点已启动！发布话题: {topic_name}')
        self.print_instructions()

    def print_instructions(self):
        print("""
==================================================
           ROS 2 纯 Python 平滑键盘控制器
==================================================
移动按键:
   q    w    e
   a    s    d
   z         c

空格键 (Space): 紧急刹车

调速按键:
   i / k : 增加 / 减少 最大线速度 (+/- 0.1 m/s)
   j / l : 增加 / 减少 最大角速度 (+/- 0.1 rad/s)

按 Ctrl+C 退出
==================================================
        """)

    def ramp_value(self, current, target, accel, dt):
        """斜坡函数：实现速度平滑插值"""
        step = accel * dt
        if target > current:
            return min(target, current + step)
        elif target < current:
            return max(target, current - step)
        return current

    def control_loop(self):
        """核心控制循环：执行加减速斜坡计算"""
        # 平滑过渡当前速度到目标速度
        self.current_lin = self.ramp_value(
            self.current_lin, self.target_lin, self.lin_accel, self.timer_period
        )
        self.current_ang = self.ramp_value(
            self.current_ang, self.target_ang, self.ang_accel, self.timer_period
        )

        # 构建并发布 Twist 消息
        twist = Twist()
        twist.linear.x = self.current_lin
        twist.angular.z = self.current_ang
        self.pub.publish(twist)


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = SmoothTeleopNode()
    try:
        while rclpy.ok():
            # 非阻塞读取按键
            key = get_key(settings)

            if key in KEY_BINDINGS:
                lin_scale, ang_scale = KEY_BINDINGS[key]  # 速度系数。在我看来起到一个正负号的作用
                node.target_lin = lin_scale * node.max_lin
                node.target_ang = ang_scale * node.max_ang
            elif key in SPEED_BINDINGS:
                lin_delta, ang_delta = SPEED_BINDINGS[key]
                node.max_lin = max(0.0, node.max_lin + lin_delta)
                node.max_ang = max(0.0, node.max_ang + ang_delta)
                node.get_logger().info(
                    f'当前设置最高速度 -> 线速度: {node.max_lin:.1f} m/s, 角速度:'
                    f' {node.max_ang:.1f} rad/s'
                )
            elif key == ' ':  # 空格急刹
                node.target_lin = 0.0
                node.target_ang = 0.0
                node.current_lin = 0.0
                node.current_ang = 0.0
            elif key == '\x03':  # Ctrl+C
                break
            else:
                # 没有任何按键按下时，目标速度自然归零（实现松手平滑刹车）
                node.target_lin = 0.0
                node.target_ang = 0.0

            # 驱动 ROS 2 节点处理一次回调
            rclpy.spin_once(node, timeout_sec=0)

    except Exception as e:
        print(e)

    finally:
        # 退出前发送全 0 速度，确保小车停下
        stop_twist = Twist()
        node.pub.publish(stop_twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
import math

class AutoExplorer(Node):
    def __init__(self):
        super().__init__('auto_explorer')
        
        # 订阅路径和里程计
        self.path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # 发布底盘控制指令 (vx, wz 或阿克曼专用话题)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.current_path = None
        self.current_pose = None
        self.lookahead_distance = 0.8  # 前视距离 (米)
        self.wheelbase = 0.4          # 你的小车轴距 (前后轮距离，根据实际改)
        
        self.timer = self.create_timer(0.05, self.control_loop) # 20Hz 控制频率
        self.get_logger().info("🎯 自研自动探索控制器已就绪！")

    def path_callback(self, msg):
        self.current_path = msg.poses

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def control_loop(self):
        if not self.current_path or not self.current_pose:
            return

        # 1. 获取当前机器人的位置和偏航角 (Yaw)
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        orientation = self.current_pose.orientation
        
        # 四元数转 Yaw 角
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # 2. 在路径上寻找一个合适的“前视目标点”
        target_point = None
        for p in self.current_path:
            px = p.pose.position.x
            py = p.pose.position.y
            dist = math.hypot(px - x, py - y)
            if dist >= self.lookahead_distance:
                target_point = (px, py)
                break
        
        if not target_point:
            # 如果路径快走完了，选最后一个点
            target_point = (self.current_path[-1].pose.position.x, self.current_path[-1].pose.position.y)

        # 3. 计算纯追踪转向角 (Pure Pursuit Geometry)
        tx, ty = target_point
        alpha = math.atan2(ty - y, tx - x) - yaw
        
        # 限制 alpha 在 [-pi, pi]
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        # 计算前轮转向弧度 delta
        steering_angle = math.atan2(2.0 * self.wheelbase * math.sin(alpha), self.lookahead_distance)

        # 4. 发布控制指令
        cmd = Twist()
        cmd.linear.x = 0.5  # 固定线速度 0.5 m/s
        # 对于标准 Twist 接口，用 angular.z 代表前轮转角或虚拟角速度
        # 如果你的底盘接收的是转向角，可以把 steering_angle 映射过去
        cmd.angular.z = steering_angle 
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AutoExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
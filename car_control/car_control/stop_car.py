#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SafeExplorer(Node):
    def __init__(self):
        super().__init__('stop_car')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # ... 其他初始化 ...

    def stop_robot(self):
        """强制给小车发全 0 速度，确保停下"""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        
        # 多发几次确保 Gazebo 彻底接收到
        for _ in range(5):
            self.cmd_pub.publish(stop_msg)
        self.get_logger().info('收到退出指令，小车已安全紧急刹车！')

def main():
    rclpy.init()
    node = SafeExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 按 Ctrl+C 时触发
        node.get_logger().info('正在退出节点...')
    finally:
        # 退出前必须执行停车逻辑！
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarTester(Node):
    def __init__(self):
        super().__init__('lidar_tester')
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

    def scan_cb(self, msg):
        ranges = np.array(msg.ranges)
        num = len(ranges)
        mid = num // 2
        q1 = num // 4
        q3 = 3 * num // 4

        print(f"\n--- 雷达总点数: {num} ---")
        print(f"索引 [0] (数组开头) 距离: {ranges[0]:.2f} 米")
        print(f"索引 [{q1}] (1/4 处) 距离: {ranges[q1]:.2f} 米")
        print(f"索引 [{mid}] (正中间) 距离: {ranges[mid]:.2f} 米")
        print(f"索引 [{q3}] (3/4 处) 距离: {ranges[q3]:.2f} 米")

def main():
    rclpy.init()
    node = LidarTester()
    rclpy.spin_once(node)  # 只打印一次就退出
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
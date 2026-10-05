import struct
import threading
import math
import serial
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        self.ser = serial.Serial(port, baud, timeout=0.01)
        self.lock = threading.Lock()
        self.get_logger().info(f'串口已打开: {port} @ {baud}')

        # 订阅 cmd_vel
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # 发布 odom
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # 航迹推算状态
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # 读串口线程
        self.running = True
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

        self.get_logger().info('已订阅 /cmd_vel，发布 /odom')

    def cmd_vel_cb(self, msg: Twist):
        self.get_logger().info(f'CMD vx={msg.linear.x:.2f} vz={msg.angular.z:.2f}')
        vx = msg.linear.x
        vz = msg.angular.z
        vx_int = max(-32768, min(32767, int(round(vx * 1000))))
        vz_int = max(-32768, min(32767, int(round(vz * 1000))))
        frame = struct.pack('>BhhB', 0x7B, vx_int, vz_int, 0x7D)
        try:
            with self.lock:
                self.ser.write(frame)
        except Exception as e:
            self.get_logger().warn(f'写串口异常: {e}')

    def read_loop(self):
        buf = bytearray()
        while self.running and rclpy.ok():
            try:
                with self.lock:
                    data = self.ser.read(64)      # 一次读 64 字节，减少锁竞争
                if not data:
                    continue
                buf.extend(data)

                # 找帧头
                while len(buf) >= 6:
                    if buf[0] == 0x5A and buf[5] == 0x5B:
                        vx_raw = struct.unpack('>h', bytes(buf[1:3]))[0]
                        vz_raw = struct.unpack('>h', bytes(buf[3:5]))[0]
                        self.publish_odom(vx_raw / 1000.0, vz_raw / 1000.0)
                        buf = buf[6:]
                    else:
                        buf = buf[1:]
                if len(buf) > 100:
                    buf = buf[-6:]
            except Exception as e:
                self.get_logger().warn(f'读串口异常: {e}')
                break

    def publish_odom(self, vx, vz):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0 or dt > 0.5:
            return

        # 航迹推算
        self.theta += vz * dt
        self.x += vx * math.cos(self.theta) * dt
        self.y += vx * math.sin(self.theta) * dt

        # 四元数
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vz
        self.odom_pub.publish(odom)

    def destroy_node(self):
        self.running = False
        with self.lock:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
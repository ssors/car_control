"""
coverage_navigator.py
读取coverage_waypoints.yaml，让小车依次导航执行全覆盖巡检
修复两处逻辑问题：
1. 脱困次数改为针对"当前单个目标点"独立计数，不再跨点累积
2. 卡死判定改为"结合真实AMCL位置是否移动"，
   而不是单纯看waypoint编号切没切换，避免正常慢速直行被误判
"""
import rclpy
import yaml
import math
import os
import time
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.srv import ClearEntireCostmap


def make_pose(navigator, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def compute_arrival_yaws(raw_points):
    yaws = []
    for i in range(len(raw_points)):
        if i == 0:
            if len(raw_points) > 1:
                dx = raw_points[1]['x'] - raw_points[0]['x']
                dy = raw_points[1]['y'] - raw_points[0]['y']
                yaw = math.atan2(dy, dx)
            else:
                yaw = 0.0
        else:
            dx = raw_points[i]['x'] - raw_points[i - 1]['x']
            dy = raw_points[i]['y'] - raw_points[i - 1]['y']
            if math.hypot(dx, dy) < 1e-3:
                yaw = yaws[-1]
            else:
                yaw = math.atan2(dy, dx)
        yaws.append(yaw)
    return yaws


class PositionTracker:
    """独立追踪车的真实AMCL位置，用于判断"是否真的没在移动"，
    而不是只看waypoint编号有没有切换"""

    def __init__(self, node):
        self.current_pos = None
        self.sub = node.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._callback, 10)

    def _callback(self, msg):
        self.current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def get_pos(self):
        return self.current_pos


def force_backup_and_clear(navigator, cmd_vel_pub, clear_local_client, clear_global_client):
    print('⚠️ 检测到疑似卡死，开始脱困流程')

    navigator.cancelTask()
    print('已请求取消当前导航任务，等待确认真正取消完成...')

    # 主动轮询确认任务真的结束了，而不是盲目sleep固定时间
    wait_start = time.time()
    while not navigator.isTaskComplete():
        time.sleep(0.2)
        if time.time() - wait_start > 5.0:
            print('等待取消超时，强制继续')
            break

    time.sleep(0.5)  # 再多给一点缓冲时间，确保controller_server彻底安静下来
    print('确认Nav2任务已停止')

    print('开始执行倒退')
    twist = Twist()
    twist.linear.x = -0.20
    twist.angular.z = 0.0

    end_time = time.time() + 3.0
    while time.time() < end_time:
        cmd_vel_pub.publish(twist)
        time.sleep(0.1)

    stop = Twist()
    cmd_vel_pub.publish(stop)
    time.sleep(0.3)
    print('倒退完成，已停止发送指令')

    if clear_local_client.wait_for_service(timeout_sec=2.0):
        clear_local_client.call_async(ClearEntireCostmap.Request())
    if clear_global_client.wait_for_service(timeout_sec=2.0):
        clear_global_client.call_async(ClearEntireCostmap.Request())
    print('已清空costmap，脱困流程完成，控制权交还Nav2')


def navigate_with_stuck_detection(navigator, poses, cmd_vel_pub,
                                    clear_local_client, clear_global_client,
                                    pos_tracker,
                                    stuck_timeout_sec=15.0,
                                    stuck_move_threshold_m=0.15,
                                    max_recovery_attempts=3):
    total = len(poses)
    i = 0
    failed_waypoints = []

    while i < total:
        remaining_poses = poses[i:]
        navigator.followWaypoints(remaining_poses)

        last_feedback_waypoint = -1
        last_progress_time = time.time()
        last_check_pos = pos_tracker.get_pos()
        recovery_attempts = 0
        is_fresh_batch = True   # 标记这是不是刚重新发起的一批，第一次反馈不算"真正推进"

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                current_wp = feedback.current_waypoint
                if current_wp != last_feedback_waypoint:
                    last_feedback_waypoint = current_wp
                    last_progress_time = time.time()
                    last_check_pos = pos_tracker.get_pos()

                    if is_fresh_batch:
                        # 这是重新发起后的第一次反馈，只是确认任务启动了，
                        # 不代表车真的往前走到了新的目标点，不清零计数
                        is_fresh_batch = False
                    else:
                        # 真正意义上切换到了下一个目标点，才清零脱困计数
                        recovery_attempts = 0

                    print(f'巡检进度: {i + current_wp}/{total}')

            now = time.time()
            if now - last_progress_time > stuck_timeout_sec:
                current_pos = pos_tracker.get_pos()
                actually_stuck = True
                if current_pos is not None and last_check_pos is not None:
                    dist_moved = math.hypot(
                        current_pos[0] - last_check_pos[0],
                        current_pos[1] - last_check_pos[1])
                    if dist_moved > stuck_move_threshold_m:
                        actually_stuck = False
                        last_progress_time = now
                        last_check_pos = current_pos
                        print(f'车仍在正常移动（{dist_moved:.2f}m），不判定为卡死，继续等待')

                if actually_stuck:
                    stuck_index = i + max(last_feedback_waypoint, 0)

                    if recovery_attempts >= max_recovery_attempts:
                        print(f'⚠️ waypoint {stuck_index} 连续{max_recovery_attempts}次脱困都失败，放弃这个点，跳到下一个')
                        failed_waypoints.append(stuck_index)
                        navigator.cancelTask()
                        time.sleep(0.5)
                        i = stuck_index + 1
                        break

                    force_backup_and_clear(navigator, cmd_vel_pub,
                                             clear_local_client, clear_global_client)

                    recovery_attempts += 1
                    print(f'当前waypoint {stuck_index} 已脱困尝试 {recovery_attempts}/{max_recovery_attempts} 次')
                    last_progress_time = time.time()
                    last_check_pos = pos_tracker.get_pos()

                    i = stuck_index
                    remaining_poses = poses[i:]
                    navigator.followWaypoints(remaining_poses)
                    last_feedback_waypoint = -1
                    is_fresh_batch = True   # 重新发起了一批，下一次反馈不算真正推进
                    continue

            time.sleep(0.2)
        else:
            i = total

    print(f'\n===== 巡检结束 =====')
    print(f'总点数: {total}')
    print(f'彻底失败/跳过的点: {len(failed_waypoints)} 个')
    if failed_waypoints:
        print(f'失败的waypoint索引: {failed_waypoints}')

    return navigator.getResult()


def main():
    rclpy.init()
    navigator = BasicNavigator()

    cmd_vel_pub = navigator.create_publisher(Twist, '/cmd_vel', 10)
    clear_local_client = navigator.create_client(
        ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
    clear_global_client = navigator.create_client(
        ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
    pos_tracker = PositionTracker(navigator)

    navigator.waitUntilNav2Active()
    print('Nav2已激活，开始加载覆盖路径')

    waypoints_path = '/home/susu/ros2/ros2_ws/src/car_control/coverage_output/coverage_waypoints.yaml'
    if not os.path.exists(waypoints_path):
        print(f'找不到路径文件: {waypoints_path}，请先运行coverage_path_generator')
        return

    with open(waypoints_path) as f:
        raw_points = yaml.safe_load(f)

    yaws = compute_arrival_yaws(raw_points)
    poses = [
        make_pose(navigator, p['x'], p['y'], yaw)
        for p, yaw in zip(raw_points, yaws)
    ]

    print(f'开始全覆盖巡检，共 {len(poses)} 个路径点（已启用改进版卡死检测）')

    result = navigate_with_stuck_detection(
        navigator, poses, cmd_vel_pub,
        clear_local_client, clear_global_client,
        pos_tracker,
        stuck_timeout_sec=15.0,
        stuck_move_threshold_m=0.15,
        max_recovery_attempts=3
    )

    if result == TaskResult.SUCCEEDED:
        print('✅ 全覆盖巡检完成')
    elif result == TaskResult.CANCELED:
        print('⚠️ 巡检被取消')
    elif result == TaskResult.FAILED:
        print('❌ 巡检失败')
    else:
        print(f'巡检结束，状态: {result}')

    navigator.lifecycleShutdown()


if __name__ == '__main__':
    main()
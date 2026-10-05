"""
coverage_path_generator.py
全覆盖路径生成 + 可视化，纯离线运行，不依赖ROS2
读取手动分割的房间结果，每个房间独立生成弓字路径 + 房间间贪心排序
去掉了二次墙角过滤（之前和SAFETY_MARGIN_M叠加导致点数暴跌），
只依赖SAFETY_MARGIN_M这一层安全边距
"""
import os
import cv2
import numpy as np
import yaml
import matplotlib.pyplot as plt


def load_map(map_yaml_path):
    """读取map.yaml + map.pgm，返回灰度图和元数据"""
    with open(map_yaml_path) as f:
        map_meta = yaml.safe_load(f)

    map_dir = os.path.dirname(os.path.abspath(map_yaml_path))
    image_path = map_meta['image']
    if not os.path.isabs(image_path):
        image_path = os.path.join(map_dir, image_path)

    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"找不到地图图片: {image_path}")

    resolution = map_meta['resolution']
    origin = map_meta['origin']
    return img, resolution, origin, map_meta


def extract_freespace(img, safety_margin_m, resolution, map_meta=None):
    """提取自由空间，正确识别细墙，并向内腐蚀留出安全边距
    额外排除标准的未知区域(灰度值205)，避免被误判成自由空间"""
    if map_meta is not None:
        free_thresh = map_meta.get('free_thresh', 0.196)
        negate = map_meta.get('negate', 0)
        if negate == 0:
            free_val_thresh = 255 * (1.0 - free_thresh)
            free_mask = (img >= free_val_thresh).astype(np.uint8) * 255
        else:
            free_val_thresh = 255 * free_thresh
            free_mask = (img <= free_val_thresh).astype(np.uint8) * 255
    else:
        free_mask = (img > 250).astype(np.uint8) * 255

    # 关键修复：明确排除未知区域(灰度值190~220这个范围，标准未知值是205附近)
    unknown_mask = (img >= 195) & (img <= 215)
    free_mask[unknown_mask] = 0

    kernel_open = np.ones((3, 3), np.uint8)
    free_mask = cv2.morphologyEx(free_mask, cv2.MORPH_OPEN, kernel_open)

    erode_pixels = max(1, int(safety_margin_m / resolution))
    erode_kernel = np.ones((3, 3), np.uint8)
    free_mask = cv2.erode(free_mask, erode_kernel, iterations=erode_pixels)

    return free_mask


def find_row_segments(row_pixels):
    """把一行里的自由空间像素切分成多个连续线段（处理房间内部家具遮挡）"""
    if len(row_pixels) == 0:
        return []
    segments = []
    start = row_pixels[0] # 记录当前线段的起始位置
    prev = row_pixels[0]  # 记录上一个访问过的像素，来判断当前像素是否和上一个连续
    for x in row_pixels[1:]:
        if x - prev > 1:
            segments.append((start, prev))
            start = x
        prev = x
    segments.append((start, prev))
    return segments


def generate_room_coverage(room_mask, row_spacing_px, min_segment_len_px=3):
    """给单个房间生成弓字形覆盖路径（像素坐标）"""
    h, w = room_mask.shape  # 房间掩码的高度和宽度
    waypoints_px = []  # 存储房间所有路径点
    rows = list(range(0, h, row_spacing_px)) # 从第0行开始，每隔row__px行取一条水平线

    for i, row in enumerate(rows):
        row_pixels = np.where(room_mask[row, :] > 0)[0]
        segments = find_row_segments(row_pixels)
        segments = [s for s in segments if (s[1] - s[0]) >= min_segment_len_px]
        if not segments:
            continue

        segments = sorted(segments, key=lambda s: s[0])
        if i % 2 == 1:
            segments = segments[::-1]  # 偶数行从左到右走，奇数行从右到左走（弓字形路径的核心）

        for left, right in segments:
            if i % 2 == 0:
                waypoints_px.append((left, row))   # 偶数行，先添加左端点
                waypoints_px.append((right, row))
            else:
                waypoints_px.append((right, row))
                waypoints_px.append((left, row))

    return waypoints_px # 返回该房间的所有路径点


def get_room_centroid(room_mask):  # 计算房间质心
    ys, xs = np.where(room_mask > 0)  # 返回所有属于该房间的像素坐标（行号，列号）
    return xs.mean(), ys.mean()


def order_rooms_by_nearest_neighbor(room_masks, start_px):
    """贪心最近邻排序，减少房间间来回穿梭"""
    remaining = list(range(len(room_masks)))
    ordered = []  # 存储排序后房间访问顺序
    current_pos = start_px

    while remaining:
        centroids = [get_room_centroid(room_masks[i]) for i in remaining]
        dists = [np.hypot(c[0] - current_pos[0], c[1] - current_pos[1]) for c in centroids]
        nearest_idx = remaining[int(np.argmin(dists))]
        ordered.append(nearest_idx)
        current_pos = get_room_centroid(room_masks[nearest_idx])
        remaining.remove(nearest_idx)

    return ordered


def pixel_to_world(px, py, resolution, origin, img_height):  # 像素转世界坐标
    wx = origin[0] + px * resolution
    wy = origin[1] + (img_height - py) * resolution
    return wx, wy


def visualize_result(img, free_mask, room_labels, num_rooms, waypoints_px, save_path): # 可视化
    """三联图：原始地图 / 房间分割结果 / 最终覆盖路径"""
    fig, axes = plt.subplots(1, 3, figsize=(20, 7))

    axes[0].imshow(img, cmap='gray')
    axes[0].set_title('原始地图')

    cmap = plt.cm.get_cmap('tab10', max(num_rooms, 1))
    colored = np.zeros((*room_labels.shape, 3))
    for rid in range(1, num_rooms + 1):
        colored[room_labels == rid] = np.array(cmap(rid - 1)[:3])
    axes[1].imshow(colored)
    axes[1].set_title(f'手动分房间结果（{num_rooms}个房间）')

    axes[2].imshow(free_mask, cmap='gray')
    if waypoints_px:
        xs = [p[0] for p in waypoints_px]
        ys = [p[1] for p in waypoints_px]
        axes[2].plot(xs, ys, 'r-', linewidth=1, alpha=0.8)
        axes[2].scatter(xs[0], ys[0], c='green', s=80, label='起点', zorder=5)
        axes[2].scatter(xs[-1], ys[-1], c='blue', s=80, label='终点', zorder=5)
        axes[2].legend()
    axes[2].set_title(f'最终覆盖路径（{len(waypoints_px)}个点）')

    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    print(f'预览图已保存到: {save_path}')
    # plt.show()  # 注释掉，避免WSL环境下弹窗卡死无法交互


def main():
    # ===== 参数配置 =====
    MAP_YAML_PATH = '/home/susu/ros2/ros2_ws/src/ros2_maps/room_latest.yaml'
    ROW_SPACING_M = 0.5   # 行间距0.5米，每0.5米走一条水平线
    SAFETY_MARGIN_M = 0.35         # 唯一的安全边距来源，不再叠加二次过滤
    OUTPUT_DIR = '/home/susu/ros2/ros2_ws/src/car_control/coverage_output'
    # ====================

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    img, resolution, origin, map_meta = load_map(MAP_YAML_PATH)
    print(f'地图尺寸: {img.shape}, 分辨率: {resolution} m/px')

    free_mask = extract_freespace(img, SAFETY_MARGIN_M, resolution, map_meta)

    # 读取手动分割结果
    room_labels_path = os.path.join(OUTPUT_DIR, 'room_labels.npy')
    if not os.path.exists(room_labels_path):
        print(f'找不到房间分割结果: {room_labels_path}')
        print('请先运行: ros2 run car_control manual_room_segmentation')
        return

    room_labels = np.load(room_labels_path)
    num_rooms = int(room_labels.max())
    print(f'已加载手动分割结果，共 {num_rooms} 个房间')

    room_masks = [(room_labels == rid).astype(np.uint8) * 255
                  for rid in range(1, num_rooms + 1)]

    row_spacing_px = max(1, int(ROW_SPACING_M / resolution))

    ys, xs = np.where(free_mask > 0)
    start_px = (xs.min(), ys[np.argmin(xs)])

    room_order = order_rooms_by_nearest_neighbor(room_masks, start_px)

    all_waypoints_px = []
    for idx in room_order:
        room_wp = generate_room_coverage(room_masks[idx], row_spacing_px)
        all_waypoints_px.extend(room_wp)

    print(f'房间访问顺序: {[i+1 for i in room_order]}')
    print(f'生成路径点数量: {len(all_waypoints_px)}')
    # 不再调用 filter_sharp_corner_waypoints，避免和SAFETY_MARGIN_M叠加导致点数暴跌

    h = img.shape[0]
    waypoints_world = [
        pixel_to_world(px, py, resolution, origin, h)
        for px, py in all_waypoints_px
    ]

    waypoints_yaml_path = os.path.join(OUTPUT_DIR, 'coverage_waypoints.yaml')
    with open(waypoints_yaml_path, 'w') as f:
        yaml.dump([{'x': float(x), 'y': float(y)} for x, y in waypoints_world], f)
    print(f'路径点已保存到: {waypoints_yaml_path}')

    preview_path = os.path.join(OUTPUT_DIR, 'coverage_preview.png')
    visualize_result(img, free_mask, room_labels, num_rooms, all_waypoints_px, preview_path)


if __name__ == '__main__':
    main()
"""
manual_room_segmentation.py
手动指定房间种子点 + 分水岭算法，精确分割出你想要的房间数量
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


# 全局变量，配合鼠标回调收集点击点
clicked_points = []


def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_points.append((x, y))
        print(f'房间{len(clicked_points)}种子点: ({x}, {y})')


def pick_room_seeds(free_mask):
    """弹出交互窗口，让你依次点击每个房间内部一点作为种子"""
    display_img = cv2.cvtColor(free_mask, cv2.COLOR_GRAY2BGR)
    window_name = 'Click each room center, press q when done'
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_callback)

    print('请依次在每个房间内部点一下(顺序不重要)，点完按 q 结束')
    while True:
        temp = display_img.copy()
        for i, (x, y) in enumerate(clicked_points):
            cv2.circle(temp, (x, y), 4, (0, 0, 255), -1)
            cv2.putText(temp, str(i + 1), (x + 6, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.imshow(window_name, temp)
        if cv2.waitKey(20) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
    return clicked_points


def watershed_segment_rooms(free_mask, seed_points):
    """用分水岭算法，以手动种子点为起点，在free_mask范围内扩展出精确房间边界"""
    h, w = free_mask.shape
    color_img = cv2.cvtColor(free_mask, cv2.COLOR_GRAY2BGR)

    markers = np.zeros((h, w), dtype=np.int32)
    for i, (x, y) in enumerate(seed_points, start=1):
        cv2.circle(markers, (x, y), 3, i, -1)

    cv2.watershed(color_img, markers)

    # watershed结果：-1是边界线，0是未处理，1..N是各房间标签
    markers[free_mask == 0] = 0   # 障碍物区域不算房间
    markers[markers < 0] = 0      # 边界线也不算房间本体

    return markers


def main():
    # ===== 参数配置，和主脚本保持一致 =====
    MAP_YAML_PATH = '/home/susu/ros2/ros2_ws/src/ros2_maps/room_latest.yaml'
    SAFETY_MARGIN_M = 0.32
    OUTPUT_DIR = '/home/susu/ros2/ros2_ws/src/car_control/coverage_output'
    # ======================================

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    img, resolution, origin, map_meta = load_map(MAP_YAML_PATH)
    print(f'地图尺寸: {img.shape}, 分辨率: {resolution} m/px')

    free_mask = extract_freespace(img, SAFETY_MARGIN_M, resolution, map_meta)

    seeds = pick_room_seeds(free_mask)
    if len(seeds) < 2:
        print('至少需要点2个房间种子点，退出')
        return

    room_labels = watershed_segment_rooms(free_mask, seeds)
    num_rooms = len(seeds)

    # 可视化检查分割效果
    cmap = plt.cm.get_cmap('tab10', num_rooms)
    colored = np.zeros((*room_labels.shape, 3))
    for rid in range(1, num_rooms + 1):
        colored[room_labels == rid] = np.array(cmap(rid - 1)[:3])

    plt.figure(figsize=(8, 8))
    plt.imshow(colored)
    plt.title(f'手动种子点分割结果（{num_rooms}个房间）')
    save_path = os.path.join(OUTPUT_DIR, 'manual_room_split.png')
    plt.savefig(save_path, dpi=150)
    print(f'分割结果已保存: {save_path}')
    plt.show()

    # 保存房间标签矩阵和种子点，供后续覆盖路径生成脚本读取
    np.save(os.path.join(OUTPUT_DIR, 'room_labels.npy'), room_labels)
    with open(os.path.join(OUTPUT_DIR, 'room_seeds.yaml'), 'w') as f:
        yaml.dump([{'x': int(x), 'y': int(y)} for x, y in seeds], f)
    print('房间标签和种子点已保存，可供coverage_path_generator后续读取使用')


if __name__ == '__main__':
    main()
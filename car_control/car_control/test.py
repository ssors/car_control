import cv2, yaml, os

MAP_YAML_PATH = '/home/susu/ros2/ros2_ws/src/ros2_maps/room_latest.yaml'
with open(MAP_YAML_PATH) as f:
    map_meta = yaml.safe_load(f)
map_dir = os.path.dirname(os.path.abspath(MAP_YAML_PATH))
img = cv2.imread(os.path.join(map_dir, map_meta['image']), cv2.IMREAD_GRAYSCALE)
resolution = map_meta['resolution']
origin = map_meta['origin']
h = img.shape[0]

points = {
    41: (2.16, -1.76), 42: (2.96, -1.76), 46: (1.16, -2.26),
    62: (2.81, -3.76), 64: (3.76, -3.76), 68: (2.01, -4.26), 74: (0.41, -4.76)
}

for idx, (wx, wy) in points.items():
    px = int((wx - origin[0]) / resolution)
    py = int(h - (wy - origin[1]) / resolution)
    val = img[py, px]
    print(f'waypoint {idx}: ({wx},{wy}) -> 灰度值 {val}')
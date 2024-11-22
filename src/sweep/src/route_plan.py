#!/usr/bin/env python3

import os
import cv2
import numpy as np
from heapq import heappush, heappop

"""
THIS IS THE (1) FIRST SCRIPT IN ROUTE MODULE
# 功能：
# 从静态地图读取数据。
# 使用 A* 算法规划路径。
# 将路径保存为实际世界的坐标，并写入文件。
"""

# 动态路径设置
script_dir = os.path.dirname(os.path.abspath(__file__))
map_path = os.path.join(script_dir, "maps/map_20241121_164920.pgm")
output_path_file = os.path.join(script_dir, "path_files/planned_path.txt")


print(f"Output path: {output_path_file}")



resolution = 0.05
origin_x, origin_y = -10.0, -10.0

# 确保路径文件夹存在
os.makedirs(os.path.dirname(output_path_file), exist_ok=True)

# 读取静态地图
map_image = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
map_array = np.array(map_image)


# 尝试加载地图
print(f"Loading map from: {map_path}")
map_image = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)

# 检查地图是否加载成功
if map_image is None:
    raise FileNotFoundError(f"Failed to load map. Check the file path: {map_path}")

# 将地图转为 numpy 数组并二值化
map_array = np.array(map_image)
map_array = (map_array > 200).astype(np.uint8)



# A* 算法实现
def a_star(start, goal, grid):
    rows, cols = grid.shape
    open_set = []
    came_from = {}
    g_score = {start: 0}
    f_score = {start: np.linalg.norm(np.array(start) - np.array(goal))}
    heappush(open_set, (f_score[start], start))

    while open_set:
        _, current = heappop(open_set)

        # 增加目标点容忍范围
        if np.linalg.norm(np.array(current) - np.array(goal)) < 5:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor] == 1:
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + np.linalg.norm(np.array(neighbor) - np.array(goal))
                    heappush(open_set, (f_score[neighbor], neighbor))

    return []

# 起点和终点（像素坐标）
start_pixel = (100, 100)  # 根据需要调整
goal_pixel = (300, 300)  # 根据需要调整

# 规划路径
path = a_star(start_pixel, goal_pixel, map_array)

# 像素坐标转实际坐标
def pixel_to_real(pixel_x, pixel_y):
    real_x = pixel_x * resolution + origin_x
    real_y = pixel_y * resolution + origin_y
    return real_x, real_y

real_path = [pixel_to_real(x, y) for x, y in path]

# 保存路径
with open(output_path_file, "w") as f:
    for point in real_path:
        f.write(f"{point[0]} {point[1]}\n")
print(f"Planned path saved to {output_path_file}")

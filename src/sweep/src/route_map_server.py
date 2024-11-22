#!/usr/bin/env python3

import os
import subprocess

"""
THIS IS THE (0) ZERO SCRIPT IN ROUTE MODULE
THIS IS A UTILITY SCRIPT
# 功能：
# 自动启动 ROS map_server 节点，加载指定的静态地图。
"""

# 动态设置地图路径
script_dir = os.path.dirname(os.path.abspath(__file__))
yaml_path = os.path.join(script_dir, "maps/map_20241121_164920.yaml")

# 检查 yaml 文件是否存在
if not os.path.exists(yaml_path):
    raise FileNotFoundError(f"YAML file not found: {yaml_path}")

# 打印 yaml 文件路径
print(f"Loading map from: {yaml_path}")

# 启动 map_server
try:
    subprocess.run(["rosrun", "map_server", "map_server", yaml_path], check=True)
except subprocess.CalledProcessError as e:
    print(f"Failed to start map_server. Error: {e}")
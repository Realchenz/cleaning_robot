#!/usr/bin/env python3

import os
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import rospy

"""
THIS IS THE (2) SECOND SCRIPT IN ROUTE MODULE
功能：
将路径可视化。
发布路径到 RViz。
"""

# 动态路径设置
script_dir = os.path.dirname(os.path.abspath(__file__))
path_file = os.path.join(script_dir, "path_files/planned_path.txt")

# 读取路径
def load_path(file_path):
    with open(file_path, "r") as f:
        lines = f.readlines()
    return [tuple(map(float, line.strip().split())) for line in lines]

real_path = load_path(path_file)

# 初始化 ROS 节点
rospy.init_node("path_visualization_node", anonymous=True)

# Publisher for the planned path
path_pub = rospy.Publisher("/planned_path", Path, queue_size=10)

def publish_path(real_path):
    """Publish a planned path as a nav_msgs/Path."""
    path_msg = Path()
    path_msg.header.frame_id = "map"
    path_msg.header.stamp = rospy.Time.now()  # Add timestamp

    for x, y in real_path:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()  # Each pose gets a timestamp
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0
        path_msg.poses.append(pose)

    # Publish the path
    rospy.loginfo("Publishing planned path...")
    path_pub.publish(path_msg)

# 发布路径
rate = rospy.Rate(1)  # 限制为 1 Hz
while not rospy.is_shutdown():
    publish_path(real_path)
    rate.sleep()

#!/usr/bin/env python3

import os
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
import numpy as np

"""
THIS IS THE (3) THIRD SCRIPT IN ROUTE MODULE
功能：
从文件读取路径。
控制机器人沿路径移动。
停止机器人后结束任务。
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
rospy.init_node("robot_motion_node")

# 发布速度
cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

# 获取机器人位置
robot_pose = None
def amcl_pose_callback(msg):
    global robot_pose
    robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)

# 检查是否到达目标点
def is_at_goal(robot_x, robot_y, goal_x, goal_y, tolerance=0.2):
    return np.sqrt((goal_x - robot_x) ** 2 + (goal_y - robot_y) ** 2) < tolerance

# 移动机器人
def move_to_point(goal_x, goal_y):
    global robot_pose
    if robot_pose is None:
        rospy.logwarn("Robot pose is not available yet.")
        return
    robot_x, robot_y = robot_pose
    twist = Twist()
    twist.linear.x = 0.2  # 调整线速度
    twist.angular.z = np.arctan2(goal_y - robot_y, goal_x - robot_x)  # 调整角速度
    cmd_vel_pub.publish(twist)

# 按路径移动
rate = rospy.Rate(10)  # 10 Hz
for goal_x, goal_y in real_path:
    while not rospy.is_shutdown():
        if robot_pose is None:
            rospy.logwarn("Waiting for robot pose...")
            rate.sleep()
            continue

        if is_at_goal(robot_pose[0], robot_pose[1], goal_x, goal_y):
            rospy.loginfo(f"Reached goal: ({goal_x}, {goal_y})")
            break

        move_to_point(goal_x, goal_y)
        rate.sleep()

# 停止机器人
rospy.loginfo("Task complete. Stopping robot...")
cmd_vel_pub.publish(Twist())
rospy.signal_shutdown("Task complete")

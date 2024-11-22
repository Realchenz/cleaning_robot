#!/usr/bin/env python3

import rospy
import subprocess
import signal
from std_msgs.msg import String

############
# MAP PATH NEEDS MODIFIED WHEN YOU CHANGE THE MAP

class AMCLController:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node("amcl_controller", anonymous=True)
        
        # 存储 AMCL 进程
        self.amcl_process = None

        # 指定地图文件的绝对路径
        self.map_file = "/home/jeff/clean_robot_119/cleaning_robot/src/sweep/src/maps/map_20241121_164920.yaml"
        
        # 订阅控制话题
        rospy.Subscriber("/control_amcl", String, self.control_callback)
        
        rospy.loginfo("AMCL Controller is running. Waiting for commands...")

    def control_callback(self, msg):
        # 接收控制命令，解析动作
        if msg.data == "start_amcl":
            self.start_amcl()
        elif msg.data == "stop_amcl":
            self.stop_amcl()
        elif msg.data == "quit":
            rospy.loginfo("Exiting AMCL Controller.")
            if self.amcl_process is not None:
                self.stop_amcl()  # 确保退出时关闭 AMCL
            rospy.signal_shutdown("User requested shutdown")

    def start_amcl(self):
        if self.amcl_process is None:
            rospy.loginfo(f"Starting AMCL with map file: {self.map_file}")
            # 启动 amcl.launch 子进程并指定地图文件
            self.amcl_process = subprocess.Popen(
                ["roslaunch", "turtlebot3_navigation", "amcl.launch", f"map_file:={self.map_file}"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
        else:
            rospy.loginfo("AMCL is already running.")

    def stop_amcl(self):
        if self.amcl_process is not None:
            rospy.loginfo("Stopping AMCL...")
            # 发送 SIGINT 信号（模拟 Ctrl+C）来终止进程
            self.amcl_process.send_signal(signal.SIGINT)
            self.amcl_process = None
        else:
            rospy.loginfo("AMCL is not running.")

    def run(self):
        rospy.spin()  # 保持节点运行，监听话题

if __name__ == "__main__":
    controller = AMCLController()
    controller.run()

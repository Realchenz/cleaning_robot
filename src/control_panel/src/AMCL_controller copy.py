#!/usr/bin/env python3
import rospy
import subprocess
import signal
import os
from std_msgs.msg import String

# UPDATE: Nov22, 2024
# Use sweep/route_map_server to control AMCL
# Because AMCL needs map_server
# Also use RVIZ to see the work

class RouteController:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node("route_controller", anonymous=True)
        
        # 存储进程
        self.route_process = None
        
        # 订阅控制话题 
        rospy.Subscriber("/control_amcl", String, self.control_callback)
        
        rospy.loginfo("Route Controller is running. Waiting for commands...")

    def control_callback(self, msg):
        # 修改为匹配GUI发送的命令
        if msg.data == "start_amcl":
            self.start_route()
        elif msg.data == "stop_amcl":
            self.stop_route()
        elif msg.data == "quit":
            rospy.loginfo("Exiting Route Controller.")
            if self.route_process is not None:
                self.stop_route()
            rospy.signal_shutdown("User requested shutdown")

    def start_route(self):
        if self.route_process is None:
            rospy.loginfo(f"Starting route_map_server using rosrun...")
            try:
                # use rosrun run route_map_server
                self.route_process = subprocess.Popen(
                    ["rosrun", "sweep", "route_map_server.py"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                rospy.loginfo("Route map server started successfully")
            except Exception as e:
                rospy.logerr(f"Failed to start route map server: {e}")
        else:
            rospy.loginfo("Route map server is already running.")


    def stop_route(self):
        if self.route_process is not None:
            rospy.loginfo("Stopping route map server...")
            try:
                # 发送终止信号给进程组
                self.route_process.send_signal(signal.SIGINT)
                self.route_process.wait(timeout=5)
                rospy.loginfo("Route map server stopped successfully")
            except subprocess.TimeoutExpired:
                rospy.logwarn("Route map server did not stop gracefully, forcing termination")
                self.route_process.kill()
            except Exception as e:
                rospy.logerr(f"Error stopping route map server: {e}")
            finally:
                self.route_process = None
        else:
            rospy.loginfo("Route map server is not running.")


    def run(self):
        rospy.spin()

if __name__ == "__main__":
    controller = RouteController()
    controller.run()
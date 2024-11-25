#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped
import tkinter as tk
from tkinter.scrolledtext import ScrolledText
import subprocess
import time
import os

# Latest Update: Nov 24, 2024
# Need Updates:
# 1. integrate route_follower in panel
# 2. modify voice control logic, use multiple words to control (eg. start exploration)
# 3. the explore will automatically stop after one hour, and automatically save the map.
# 4. add all button voice command in panel_gui

class ControlPanelGUI:
    def __init__(self, root):
        rospy.init_node("control_panel_gui_node", anonymous=True)

        # ROS Publishers
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.control_move_base_pub = rospy.Publisher("/control_move_base", String, queue_size=10)
        self.control_explore_pub = rospy.Publisher("/control_explore", String, queue_size=10)
        self.velocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)
        self.control_gmapping_pub = rospy.Publisher("/control_gmapping", String, queue_size=10)
        self.control_amcl_pub = rospy.Publisher("/control_amcl", String, queue_size=10)
        self.control_route_show_pub = rospy.Publisher("/control_route_show", String, queue_size=10)
        self.control_route_follow_pub = rospy.Publisher("/control_route_follow", String, queue_size=10)

        # ROS Subscribers
        rospy.Subscriber("voice_commands", String, self.voice_callback)

        # Main GUI Layout
        root.title("Control Panel")
        root.geometry("750x800") # 800 (width) x 800 (height)

        # Button Frame
        button_frame = tk.Frame(root)
        button_frame.pack(side="top", fill="x", padx=5, pady=5)

        # Voice Recognition Title and Box
        voice_frame = tk.Frame(root)
        voice_frame.pack(side="top", fill="x", padx=5, pady=5)
        tk.Label(voice_frame, text="Voice Recognition:", font=("Arial", 12, "bold")).pack(anchor="w")
        self.voice_text = tk.Text(voice_frame, height=4, wrap="word", bg="#f4f4f4")
        self.voice_text.pack(fill="x")
        self.voice_text.insert("1.0", "Voice Recognition Output:\n")
        self.voice_text.configure(state="disabled")

        # Run Hint Title and Box
        hint_frame = tk.Frame(root)
        hint_frame.pack(side="top", fill="x", padx=5, pady=5)
        tk.Label(hint_frame, text="Run Hint and Instructions:", font=("Arial", 12, "bold")).pack(anchor="w")
        self.run_hint = tk.Text(hint_frame, height=4, wrap="word", bg="#f4f4f4")
        self.run_hint.pack(fill="x")
        self.run_hint.insert("1.0", "Run Hint Output:\n")
        self.run_hint.configure(state="disabled")

        # ROS Log Title and Output Box
        log_frame = tk.Frame(root)
        log_frame.pack(side="top", fill="both", expand=True, padx=5, pady=5)
        tk.Label(log_frame, text="ROS Log Output for Developers:", font=("Arial", 12, "bold")).pack(anchor="w")
        self.log_text = ScrolledText(log_frame, wrap="word", bg="black", fg="white")
        self.log_text.pack(fill="both", expand=True)
        self.log_text.insert("1.0", "ROS Log Output:\n")
        self.log_text.configure(state="disabled")

        # Buttons - Mapping Functions
        tk.Label(button_frame, text="Mapping Functions:", font=("Arial", 12, "bold")).grid(row=0, column=0, columnspan=2, pady=5, sticky="w")
        tk.Button(button_frame, text="Start SLAM", command=self.start_slam).grid(row=1, column=0, padx=5, pady=5)
        tk.Button(button_frame, text="Start Exploration", command=self.start_exploration).grid(row=1, column=1, padx=5, pady=5)
        tk.Button(button_frame, text="Stop Exploration", command=self.stop_exploration).grid(row=1, column=2, padx=5, pady=5)
        tk.Button(button_frame, text="Save Map", command=self.save_map).grid(row=1, column=3, padx=5, pady=5)
        tk.Button(button_frame, text="Finish Mapping", command=self.stop_slam).grid(row=1, column=4, padx=5, pady=5)

        # Buttons - Route Analyze Functions
        tk.Label(button_frame, text="Route Analyze Functions:", font=("Arial", 12, "bold")).grid(row=2, column=0, columnspan=2, pady=5, sticky="w")
        tk.Button(button_frame, text="Load Map", command=self.start_amcl).grid(row=3, column=0, padx=5, pady=5)
        tk.Button(button_frame, text="Anaylyze Route", command=self.analyze_route).grid(row=3, column=1, padx=5, pady=5)
        tk.Button(button_frame, text="Show Route", command=self.start_route_show).grid(row=3, column=2, padx=5, pady=5)
        tk.Button(button_frame, text="Refresh Route", command=self.refresh_route_show).grid(row=3, column=3, padx=5, pady=5)
        tk.Button(button_frame, text="Finish Analyzing", command=self.stop_amcl).grid(row=3, column=4, padx=5, pady=5)

        # Buttons - Cleaning Functions (Follow the Route)
        # tk.Label(button_frame, text="Cleaning Functions", font=("Arial", 12, "bold")).grid(row=4, column=0, columnspan=2, pady=5, sticky="w")
        # tk.Button(button_frame, text="Start Cleaning", command=self.start_cleaning).grid(row=5, column=0, padx=5, pady=5)
        # tk.Button(button_frame, text="Stop Cleaning", command=self.stop_cleaning).grid(row=5, column=1, padx=5, pady=5)

        # 修改后的代码:
        tk.Label(button_frame, text="Cleaning Functions", font=("Helvetica", 12, "bold")).grid(row=4, column=0, columnspan=2, pady=5, sticky="w")
        tk.Button(button_frame, text="Start Cleaning", command=self.start_cleaning, font=("Helvetica", 10)).grid(row=5, column=0, padx=5, pady=5)
        tk.Button(button_frame, text="Stop Cleaning", command=self.stop_cleaning, font=("Helvetica", 10)).grid(row=5, column=1, padx=5, pady=5)


        # Quit Program 按钮靠右对齐
        tk.Button(button_frame, text="Quit Program", command=self.quit_program).grid(row=5, column=4, padx=5, pady=5, sticky="e")

        ######################################       
        # Control Moving
        # 在"Quit Program"按钮后面添加一个分隔
        tk.Label(button_frame, text="").grid(row=5, column=0, pady=10)  # 添加一些垂直空间

        # 添加机器人控制按钮组（放在最下面一行，和其他按钮分开）
        control_frame = tk.LabelFrame(button_frame, text="Robot Control", font=("Arial", 12, "bold"), padx=10, pady=5)
        control_frame.grid(row=6, column=0, columnspan=6, pady=10, sticky="ew")  # 跨越所有列

        # 方向按钮
        self.btn_up = tk.Button(control_frame, text="↑", command=self.move_forward, width=3)
        self.btn_left = tk.Button(control_frame, text="←", command=self.move_left, width=3)
        self.btn_stop = tk.Button(control_frame, text="■", command=self.stop_robot, width=3)
        self.btn_right = tk.Button(control_frame, text="→", command=self.move_right, width=3)
        self.btn_down = tk.Button(control_frame, text="↓", command=self.move_backward, width=3)

        # 布局方向按钮（保持在中间）
        self.btn_up.grid(row=0, column=1, pady=2)
        self.btn_left.grid(row=1, column=0, padx=2)
        self.btn_stop.grid(row=1, column=1)
        self.btn_right.grid(row=1, column=2, padx=2)
        self.btn_down.grid(row=2, column=1, pady=2)

        # Must make sure stop SLAM before running ACML 
        self.check_slam_running = False

    def log(self, message):
        """Append message to the log output box."""
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"{message}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def hint(self, message):
        """Show hint in the run hint box."""
        self.run_hint.configure(state="normal")
        self.run_hint.insert("end", f"{message}\n")
        self.run_hint.see("end")
        self.run_hint.configure(state="disabled")

    def voice_callback(self, msg):
        """Handle voice commands."""
        command = msg.data.lower()
        self.voice_text.configure(state="normal")
        self.voice_text.insert("end", f"Voice Command: {command}\n")
        self.voice_text.see("end")
        self.voice_text.configure(state="disabled")

        if "start exploration" in command:
            self.start_exploration()
        elif "stop exploration" in command:
            self.stop_exploration()
        # elif "quit program" in command:
        #     self.quit_program()

    def start_slam(self):
        self.log("Starting SLAM...")
        self.hint("Button Pressed: Start SLAM")
        self.control_gmapping_pub.publish("start_gmapping")
        self.check_slam_running = True

    def stop_slam(self):
        self.log("Stopping SLAM...")
        self.hint("Button Pressed: Stop SLAM")
        self.control_gmapping_pub.publish("stop_gmapping")
        self.check_slam_running = False
        

    def start_exploration(self):
        self.log("Starting Exploration...")
        self.hint("Button Pressed: Start Exploration")
        self.control_explore_pub.publish("start_explore")
        self.control_move_base_pub.publish("start_move_base")

    def stop_exploration(self):
        self.log("Stopping Exploration...")
        self.hint("Button Pressed: Stop Exploration")
        self.control_explore_pub.publish("stop_explore")
        self.control_move_base_pub.publish("stop_move_base")
        self.stop_robot()

    def save_map(self):
        map_path = "~/clean_robot_119/cleaning_robot/maps/map_" + time.strftime("%Y%m%d_%H%M%S")
        self.log(f"Saving map to {map_path}...")
        try:
            subprocess.call(["rosrun", "map_server", "map_saver", "-f", os.path.expanduser(map_path)])
            self.log(f"Map saved to {map_path}")
        except Exception as e:
            self.log(f"Failed to save map: {e}")

    def start_amcl(self):
        self.log("Starting AMCL...")
        # 检查SLAM状态，如果在运行就提示错误
        if self.check_slam_running==True:  # 需要添加检查函数
            self.log("Please stop SLAM first")
            return
        self.hint("Button Pressed: Start AMCL")
        self.control_amcl_pub.publish("start_amcl")

    def stop_amcl(self):
        self.log("Stopping AMCL...")
        self.hint("Button Pressed: Stop AMCL")
        # at same time stop route_show here
        self.stop_route_show()  # 先停止route_show
        self.control_amcl_pub.publish("stop_amcl")



    def stop_robot(self):
        self.log("Stopping Robot...")
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.velocity_pub.publish(stop_msg)
        self.cancel_pub.publish(GoalID())

    def start_route_show(self):
        self.control_route_show_pub.publish("start_show")
        self.log("Starting route show...")

    def stop_route_show(self):
        self.control_route_show_pub.publish("stop_show")
        self.log("Stopping route show...")

    def refresh_route_show(self):
        self.control_route_show_pub.publish("stop_show")
        self.log("Stopping route show to refresh")
        self.control_route_show_pub.publish("start_show")
        self.log("Starting route show... Refresh complete.")



    def analyze_route(self):
        """调用route_plan.py进行路径规划"""
        # (route_plan) this is one time script, so do not need a controller
        # Also, this script can update while the panel is running!
        self.log("Starting route analysis...")
        try:
            # 运行路径规划脚本
            # 使用rosrun启动route_show.py
            subprocess.Popen(
                    ["rosrun", "sweep", "route_plan.py"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
            self.log("Route analysis completed successfully")
            self.log("But the processing might take some time,")
            self.log("So please wait for at least 10 seconds before showing the map!")
        except Exception as e:
            self.log(f"Unexpected error during route analysis: {e}")


    def quit_program(self):
        self.log("Quitting Program...")
        try:
            subprocess.call(["rosnode", "kill", "-a"])  # 关闭所有ROS节点
        except Exception as e:
            self.log(f"Failed to kill ROS nodes: {e}")
        rospy.signal_shutdown("User requested shutdown")
        root.quit()


    # 添加控制函数
    def move_forward(self):
        """Move robot forward"""
        twist = Twist()
        twist.linear.x = 0.2  # 前进速度 m/s
        self.velocity_pub.publish(twist)
        self.log("Moving forward...")

    def move_backward(self):
        """Move robot backward"""
        twist = Twist()
        twist.linear.x = -0.2  # 后退速度 m/s
        self.velocity_pub.publish(twist)
        self.log("Moving backward...")

    def move_left(self):
        """Rotate robot left"""
        twist = Twist()
        twist.angular.z = 0.5  # 左转角速度 rad/s
        self.velocity_pub.publish(twist)
        self.log("Turning left...")

    def move_right(self):
        """Rotate robot right"""
        twist = Twist()
        twist.angular.z = -0.5  # 右转角速度 rad/s
        self.velocity_pub.publish(twist)
        self.log("Turning right...")

    def start_cleaning(self):
        self.control_route_follow_pub.publish("start_route_follow")
        self.log("Starting route follow...")
        self.log("If you are using RViz, and already open route_show routes,")
        self.log("Please remember switch the MarkerArray's topic to switch to route_follow markers.")
        self.log("Please wait for the initilization for about 10 seconds.")

    def stop_cleaning(self):
        self.control_route_follow_pub.publish("stop_route_follow")
        self.log("Stop route follow.")


if __name__ == "__main__":
    root = tk.Tk()
    gui = ControlPanelGUI(root)
    root.mainloop()

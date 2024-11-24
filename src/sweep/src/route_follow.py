#!/usr/bin/env python3
import rospy
import yaml
import os
import math
from geometry_msgs.msg import Twist, PoseStamped
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import tf
from tf.transformations import euler_from_quaternion
import numpy as np

# Create Date: Nov 24, 2024
# Problems:
# 1. save too many files
# 2. the following sequence is not the same as the route_plan and route_show desined
# 3. The nearby points judgement is not what I mean by judge "precise position", i want to use AMCL to judge

class MapData:
    OBSTACLE = -1
    UNVISITED = 0
    VISITED = 1

class RouteFollower:
    def __init__(self):
        rospy.init_node('route_follower', anonymous=True)
        
        # 参数设置
        self.SEARCH_RADIUS = 0.3      # 定位精度检查的搜索半径
        self.MAX_NEARBY_POINTS = 12     # 允许的最大临近点数
        self.POSITION_TOLERANCE = 0.1   # 到达目标点的位置误差
        self.ANGLE_TOLERANCE = 0.1     # 角度误差
        self.LINEAR_SPEED = 0.2        # 线速度
        self.ANGULAR_SPEED = 0.3       # 角速度
        
        # 数据存储
        self.path_points = []
        self.current_pose = None
        
        # 创建发布器和订阅器
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.status_pub = rospy.Publisher('/route_follower/status', String, queue_size=1)
        
        # 订阅AMCL位姿
        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        
        # TF监听器
        self.tf_listener = tf.TransformListener()
        
        # 加载路径文件
        self.path_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pathfiles")
        if not os.path.exists(self.path_dir):
            os.makedirs(self.path_dir)
            
        self.load_latest_path()
        rospy.loginfo("Route follower initialized")
        
    def load_latest_path(self):
        """加载最新的路径文件"""
        try:
            path_files = [f for f in os.listdir(self.path_dir) 
                         if f.startswith('coverage_path_') and f.endswith('.yaml')]
            if not path_files:
                rospy.logwarn("No path files found")
                return False
                
            latest_file = sorted(path_files)[-1]
            path_file = os.path.join(self.path_dir, latest_file)
            rospy.loginfo(f"Loading path from {path_file}")
            
            with open(path_file, 'r') as f:
                data = yaml.safe_load(f)
                self.path_points = data['path_points']
                rospy.loginfo(f"Loaded {len(self.path_points)} path points")
            return True
                
        except Exception as e:
            rospy.logerr(f"Error loading path file: {str(e)}")
            return False

    def amcl_callback(self, msg):
        """处理AMCL位姿更新"""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        # 转换四元数为欧拉角
        _, _, yaw = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_pose = (position.x, position.y, yaw)

    def get_robot_pose(self):
        """获取机器人当前位姿"""
        if self.current_pose is None:
            rospy.logwarn("Waiting for AMCL pose...")
            return None
        return self.current_pose

    def check_localization_accuracy(self):
        """检查定位精度"""
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return False, "Unable to get robot pose"
        
        nearby_points = []
        for point in self.path_points:
            if point['status'] == MapData.UNVISITED:
                dist = ((point['world_x'] - robot_pose[0])**2 + 
                       (point['world_y'] - robot_pose[1])**2)**0.5
                if dist < self.SEARCH_RADIUS:
                    nearby_points.append(point)
        
        if len(nearby_points) > self.MAX_NEARBY_POINTS:
            return False, f"Too many possible points nearby ({len(nearby_points)}). Please improve localization."
            
        if not nearby_points:
            return False, "No path points nearby. Please move to the path."
            
        return True, "Localization accuracy sufficient"

    def find_nearest_unvisited_point(self, robot_pose):
        """找到最近的未访问点"""
        nearest_point = None
        min_distance = float('inf')
        
        for point in self.path_points:
            if point['status'] == MapData.UNVISITED:
                dist = ((point['world_x'] - robot_pose[0])**2 + 
                       (point['world_y'] - robot_pose[1])**2)**0.5
                if dist < min_distance:
                    min_distance = dist
                    nearest_point = point
                    
        return nearest_point

    def move_to_point(self, target_point):
        """移动到目标点"""
        if not self.get_robot_pose():
            return False
            
        rate = rospy.Rate(10)  # 10Hz
        cmd_vel = Twist()
        
        start_time = rospy.Time.now()
        timeout = rospy.Duration(30.0)  # 30秒超时
        
        while not rospy.is_shutdown():
            current_pose = self.get_robot_pose()
            if not current_pose:
                continue
                
            # 检查是否超时
            if (rospy.Time.now() - start_time) > timeout:
                rospy.logwarn("Move to point timeout")
                self.stop_robot()
                return False
                
            # 计算距离和角度
            dx = target_point['world_x'] - current_pose[0]
            dy = target_point['world_y'] - current_pose[1]
            distance = (dx**2 + dy**2)**0.5
            target_angle = math.atan2(dy, dx)
            
            # 如果到达目标点
            if distance < self.POSITION_TOLERANCE:
                self.stop_robot()
                return True
                
            # 计算角度差
            angle_diff = target_angle - current_pose[2]
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            
            # 根据角度差决定运动方式
            if abs(angle_diff) > self.ANGLE_TOLERANCE:
                # 原地旋转到正确方向
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = self.ANGULAR_SPEED if angle_diff > 0 else -self.ANGULAR_SPEED
            else:
                # 保持方向的同时前进
                cmd_vel.linear.x = min(self.LINEAR_SPEED, distance)
                cmd_vel.angular.z = 0.5 * angle_diff  # 使用比例控制保持方向
                
            self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()
            
        self.stop_robot()
        return False

    def stop_robot(self):
        """停止机器人"""
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

    def update_point_status(self, point_id, new_status):
        """更新路径点状态"""
        for point in self.path_points:
            if point['id'] == point_id:
                point['status'] = new_status
                return True
        return False

    def save_current_progress(self):
        """保存当前进度"""
        try:
            timestamp = rospy.Time.now().to_sec()
            progress_file = os.path.join(self.path_dir, f"route_progress_{timestamp}.yaml")
            
            progress_data = {
                'timestamp': timestamp,
                'path_points': self.path_points
            }
            
            with open(progress_file, 'w') as f:
                yaml.dump(progress_data, f)
                
            rospy.loginfo(f"Progress saved to {progress_file}")
            
        except Exception as e:
            rospy.logerr(f"Failed to save progress: {e}")

    def follow_route(self):
        """主控制循环"""
        rospy.loginfo("Starting route following...")
        rate = rospy.Rate(1)  # 1Hz检查频率
        
        while not rospy.is_shutdown():
            # 1. 检查定位精度
            accuracy_ok, msg = self.check_localization_accuracy()
            if not accuracy_ok:
                rospy.logwarn(msg)
                self.status_pub.publish(msg)
                rate.sleep()
                continue
            
            # 2. 获取当前位置
            robot_pose = self.get_robot_pose()
            if not robot_pose:
                continue
                
            # 3. 找到最近的未访问点
            next_point = self.find_nearest_unvisited_point(robot_pose)
            if not next_point:
                rospy.loginfo("All points visited!")
                self.status_pub.publish("Route completed")
                break
                
            # 4. 移动到该点
            rospy.loginfo(f"Moving to point {next_point['id']}")
            self.status_pub.publish(f"Moving to point {next_point['id']}")
            
            if self.move_to_point(next_point):
                # 5. 更新点的状态
                self.update_point_status(next_point['id'], MapData.VISITED)
                # 6. 保存进度
                self.save_current_progress()
            else:
                rospy.logwarn(f"Failed to reach point {next_point['id']}")
                self.status_pub.publish(f"Failed to reach point {next_point['id']}")
            
            rate.sleep()
        
        self.stop_robot()
        rospy.loginfo("Route following completed")

    def run(self):
        """运行路径跟随器"""
        # 等待初始定位
        rospy.sleep(2)  # 给AMCL一些时间进行初始定位
        
        if not self.path_points:
            rospy.logerr("No path points loaded. Cannot start route following.")
            return
            
        try:
            self.follow_route()
        except KeyboardInterrupt:
            self.stop_robot()
            rospy.loginfo("Route following interrupted by user")
        except Exception as e:
            self.stop_robot()
            rospy.logerr(f"Route following error: {str(e)}")
        finally:
            self.save_current_progress()

if __name__ == "__main__":
    try:
        follower = RouteFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass
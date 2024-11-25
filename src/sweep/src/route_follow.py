#!/usr/bin/env python3
import rospy
import yaml
import os
import math
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, ColorRGBA
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
from geometry_msgs.msg import Point, PoseStamped

# Update: Nov 24, 2024
# Notice: 
# 1. Didn't integrated into panel yet, adjust and debugging...
# 2. Need manually switch RViz topic to route_status_visualization

class MapData:
    OBSTACLE = -1    # 障碍物点
    UNVISITED = 0    # 未访问的路径点
    VISITED = 1      # 已访问的路径点

class RouteFollower:
    def __init__(self):
        rospy.init_node('route_follower', anonymous=True)
        
        # 参数设置
        self.POSITION_TOLERANCE = 0.1   # 到达目标点的位置误差
        self.ANGLE_TOLERANCE = 0.1      # 角度误差
        self.LINEAR_SPEED = 0.2         # 线速度
        self.ANGULAR_SPEED = 0.3        # 角速度
        self.SAFETY_DISTANCE = 0.3      # 安全距离阈值
        self.SAMPLING_INTERVAL = 5      # 采样间隔，与route_plan保持一致
        
        # 数据存储
        self.path_points = []
        self.path_connections = []      # 存储路径连接关系
        self.current_pose = None
        self.current_path_index = None
        self.obstacle_points = []
        self.current_path = Path()      # 当前正在执行的路径
        
        # 创建发布器和订阅器
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.status_pub = rospy.Publisher('/route_status', String, queue_size=1)
        self.visualization_pub = rospy.Publisher('/route_status_visualization', MarkerArray, queue_size=1, latch=True)
        self.path_pub = rospy.Publisher('/current_path', Path, queue_size=1, latch=True)
        
        # 订阅位姿和激光雷达数据
        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        # 加载路径文件
        self.path_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pathfiles")
        if not os.path.exists(self.path_dir):
            os.makedirs(self.path_dir)
        
        self.load_latest_path()
        rospy.loginfo("Route follower initialized")

    def load_latest_path(self):
        """加载最新的路径文件并重置所有点状态"""
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
                # 重置所有点的状态为未访问
                for point in self.path_points:
                    point['status'] = MapData.UNVISITED
                # 重建路径连接关系
                self.build_path_connections()
                rospy.loginfo(f"Loaded {len(self.path_points)} path points")
                
            # 删除旧的状态文件（如果存在）
            status_file = os.path.join(self.path_dir, "path_status.yaml")
            if os.path.exists(status_file):
                os.remove(status_file)
                
            return True
                
        except Exception as e:
            rospy.logerr(f"Error loading path file: {str(e)}")
            return False

    def build_path_connections(self):
        """构建路径连接关系，与route_plan保持一致"""
        self.path_connections = []
        visited = set()
        
        if not self.path_points:
            return
            
        current_point = self.path_points[0]
        visited.add(0)
        
        while len(visited) < len(self.path_points):
            best_distance = float('inf')
            best_next_point = None
            best_next_idx = None
            
            # 找到最近的未访问相邻点
            for i, point in enumerate(self.path_points):
                if i in visited:
                    continue
                
                # 检查是否相邻（使用采样间隔）
                dx = abs(current_point['grid_x'] - point['grid_x'])
                dy = abs(current_point['grid_y'] - point['grid_y'])
                
                if not ((dx == self.SAMPLING_INTERVAL and dy == 0) or 
                        (dx == 0 and dy == self.SAMPLING_INTERVAL)):
                    continue
                
                distance = math.hypot(
                    current_point['grid_x'] - point['grid_x'],
                    current_point['grid_y'] - point['grid_y']
                )
                
                if distance < best_distance:
                    best_distance = distance
                    best_next_point = point
                    best_next_idx = i
            
            if best_next_point is None:
                # 如果找不到相邻点，从未访问点中选择一个新的起点
                unvisited = set(range(len(self.path_points))) - visited
                if unvisited:
                    next_start_idx = min(unvisited)
                    current_point = self.path_points[next_start_idx]
                    visited.add(next_start_idx)
                    continue
                else:
                    break
            
            self.path_connections.append((current_point['id'], best_next_point['id']))
            visited.add(best_next_idx)
            current_point = best_next_point
        
        rospy.loginfo(f"Built {len(self.path_connections)} path connections")

    def get_next_planned_point(self, current_point_id):
        """根据规划的路径获取下一个点"""
        # 在路径连接中查找下一个点
        for start_id, end_id in self.path_connections:
            if start_id == current_point_id:
                # 找到目标点的完整信息
                for point in self.path_points:
                    if point['id'] == end_id and point['status'] == MapData.UNVISITED:
                        # 确保下一个点是可访问的
                        if self.check_path_safety(point):
                            return point
        return None

    def find_nearest_accessible_point(self):
        """找到最近的可访问的未访问点"""
        if not self.current_pose:
            return None
            
        min_distance = float('inf')
        nearest_point = None
        
        for point in self.path_points:
            if point['status'] != MapData.UNVISITED:
                continue
                
            distance = math.hypot(
                point['world_x'] - self.current_pose[0],
                point['world_y'] - self.current_pose[1]
            )
            
            if distance < min_distance and self.check_path_safety(point):
                min_distance = distance
                nearest_point = point
                
        return nearest_point

    def amcl_callback(self, msg):
        """处理AMCL位姿更新"""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_pose = (position.x, position.y, yaw)

    def laser_callback(self, scan_msg):
        """处理激光雷达数据"""
        if not self.current_pose:
            return

        self.obstacle_points = []
        angle = scan_msg.angle_min
        
        for r in scan_msg.ranges:
            if scan_msg.range_min <= r <= scan_msg.range_max:
                # 转换到地图坐标系
                x = self.current_pose[0] + r * math.cos(angle + self.current_pose[2])
                y = self.current_pose[1] + r * math.sin(angle + self.current_pose[2])
                self.obstacle_points.append((x, y))
            angle += scan_msg.angle_increment

    def check_path_safety(self, target_point):
        """检查到目标点的路径是否安全"""
        if not self.current_pose or not self.obstacle_points:
            return True
            
        path_start = np.array([self.current_pose[0], self.current_pose[1]])
        path_end = np.array([target_point['world_x'], target_point['world_y']])
        path_vector = path_end - path_start
        path_length = np.linalg.norm(path_vector)
        
        if path_length == 0:
            return True
            
        for obs_point in self.obstacle_points:
            obs_vector = np.array(obs_point) - path_start
            # 计算投影
            projection = np.dot(obs_vector, path_vector) / path_length
            
            if 0 <= projection <= path_length:
                # 计算垂直距离
                distance = abs(np.cross(path_vector, obs_vector)) / path_length
                if distance < self.SAFETY_DISTANCE:
                    return False
        
        return True

    def publish_point_status(self):
        """发布所有点的状态用于可视化"""
        point_markers = MarkerArray()
        
        for i, point in enumerate(self.path_points):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "path_points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = point['world_x']
            marker.pose.position.y = point['world_y']
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = marker.scale.y = marker.scale.z = 0.1
            
            if point['status'] == MapData.VISITED:
                marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)  # 绿色
            elif point['status'] == MapData.OBSTACLE:
                marker.color = ColorRGBA(0.0, 0.0, 0.0, 0.8)  # 黑色
            else:
                marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.8)  # 红色
                
            point_markers.markers.append(marker)

        # 添加路径连接的可视化
        for i, (start_id, end_id) in enumerate(self.path_connections):
            start_point = next(p for p in self.path_points if p['id'] == start_id)
            end_point = next(p for p in self.path_points if p['id'] == end_id)
            
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "path_connections"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # 设置箭头起点和终点
            marker.points = [
                Point(x=start_point['world_x'], y=start_point['world_y'], z=0),
                Point(x=end_point['world_x'], y=end_point['world_y'], z=0)
            ]
            
            # 设置箭头大小
            marker.scale.x = 0.02  # 箭身宽度
            marker.scale.y = 0.04  # 箭头宽度
            marker.scale.z = 0.01  # 箭头高度
            
            # 设置箭头颜色（蓝色）
            marker.color = ColorRGBA(0.3, 0.3, 1.0, 0.8)
            
            point_markers.markers.append(marker)
        
        self.visualization_pub.publish(point_markers)

    def publish_current_path(self, target_point):
        """发布当前路径用于可视化"""
        if not self.current_pose:
            return
            
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        # 添加起点（当前位置）
        start_pose = PoseStamped()
        start_pose.header = path_msg.header
        start_pose.pose.position.x = self.current_pose[0]
        start_pose.pose.position.y = self.current_pose[1]
        start_pose.pose.orientation.w = 1.0
        path_msg.poses.append(start_pose)

        # 添加目标点
        target_pose = PoseStamped()
        target_pose.header = path_msg.header
        target_pose.pose.position.x = target_point['world_x']
        target_pose.pose.position.y = target_point['world_y']
        target_pose.pose.orientation.w = 1.0
        path_msg.poses.append(target_pose)

        self.path_pub.publish(path_msg)

    def move_to_point(self, target_point):
        """移动到目标点"""
        if not self.current_pose:
            return False
            
        rate = rospy.Rate(10)  # 10Hz
        cmd_vel = Twist()
        
        start_time = rospy.Time.now()
        timeout = rospy.Duration(30.0)  # 30秒超时
        
        while not rospy.is_shutdown():
            if not self.current_pose:
                continue
                
            # 检查是否超时
            if (rospy.Time.now() - start_time) > timeout:
                rospy.logwarn("Move to point timeout")
                self.stop_robot()
                return False
                
            # 检查路径安全性
            if not self.check_path_safety(target_point):
                rospy.logwarn("Path blocked by obstacle")
                self.stop_robot()
                return False
                
            # 计算距离和角度
            dx = target_point['world_x'] - self.current_pose[0]
            dy = target_point['world_y'] - self.current_pose[1]
            distance = math.hypot(dx, dy)
            target_angle = math.atan2(dy, dx)
            
            # 如果到达目标点
            if distance < self.POSITION_TOLERANCE:
                self.stop_robot()
                return True
                
            # 计算角度差
            angle_diff = target_angle - self.current_pose[2]
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

    def check_localization_accuracy(self):
        """检查AMCL定位精度"""
        if not self.current_pose:
            return False, "No pose data"
        return True, "Localization accuracy sufficient"

    def update_point_status(self, point_id, new_status=MapData.VISITED):
        """更新点的状态"""
        for point in self.path_points:
            if point['id'] == point_id:
                point['status'] = new_status
                self.publish_point_status()
                self.save_points_status()
                return True
        return False

    def save_points_status(self):
        """保存路径点状态"""
        status_file = os.path.join(self.path_dir, "path_status.yaml")
        status_data = {
            'timestamp': rospy.get_time(),
            'points_status': [
                {
                    'id': point['id'],
                    'status': point['status']
                }
                for point in self.path_points
            ]
        }
        
        try:
            with open(status_file, 'w') as f:
                yaml.dump(status_data, f)
        except Exception as e:
            rospy.logwarn(f"Error saving status file: {str(e)}")

    def follow_route(self):
        """主控制循环"""
        rospy.loginfo("Starting route following...")
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            # 检查定位
            accuracy_ok, msg = self.check_localization_accuracy()
            if not accuracy_ok:
                rospy.logwarn(msg)
                rate.sleep()
                continue
                
            # 如果没有当前目标点，寻找一个新的
            if self.current_path_index is None:
                next_point = self.find_nearest_accessible_point()
                if not next_point:
                    rospy.loginfo("No more accessible points. Mission complete!")
                    break
                self.current_path_index = next_point['id']
            
            # 获取当前目标点
            current_target = None
            for point in self.path_points:
                if point['id'] == self.current_path_index:
                    current_target = point
                    break
            
            if not current_target:
                rospy.logwarn("Current target point not found")
                self.current_path_index = None
                continue
            
            # 检查路径安全性
            if not self.check_path_safety(current_target):
                rospy.logwarn("Path to target blocked")
                self.update_point_status(current_target['id'], MapData.OBSTACLE)
                self.current_path_index = None
                continue
            
            # 发布当前路径
            self.publish_current_path(current_target)
            
            # 移动到目标点
            if self.move_to_point(current_target):
                # 更新点状态
                self.update_point_status(current_target['id'])
                
                # 优先获取规划路径中的下一个点
                next_point = self.get_next_planned_point(current_target['id'])
                
                if not next_point:
                    # 如果规划路径中没有下一个点，寻找最近的可达点
                    next_point = self.find_nearest_accessible_point()
                
                if next_point:
                    self.current_path_index = next_point['id']
                else:
                    rospy.loginfo("No more points to visit")
                    break
            else:
                rospy.logwarn("Failed to reach target point")
                self.update_point_status(current_target['id'], MapData.OBSTACLE)
                self.current_path_index = None
            
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
            # 开始跟随路径
            self.follow_route()
        except KeyboardInterrupt:
            self.stop_robot()
            rospy.loginfo("Route following interrupted by user")
        except Exception as e:
            self.stop_robot()
            rospy.logerr(f"Route following error: {str(e)}")
            raise  # 重新抛出异常以便调试

if __name__ == "__main__":
    try:
        follower = RouteFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID       # to publish cancel exploring command
from geometry_msgs.msg import PoseStamped   # to publish goal command


def main():
    rospy.init_node("control_node", anonymous=True)
    
    rospy.sleep(1) # wait for prepare fully

    # 创建发布者，发布到 /move_base_simple/goal 话题
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)


    # Publisher for control move_base
    control_move_base_pub = rospy.Publisher("/control_move_base", String, queue_size=10)

    # Publisher for control explore
    control_explore_pub = rospy.Publisher("/control_explore", String, queue_size=10)


    
    # Publisher for velocity commands
    velocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    
    # Publisher for move_base cancel command
    cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)

    print("============= CONTROL PANEL =============")
    print("PRESS '0' TO GO BACK TO THE BASE POINT")
    print("PRESS '1' TO START EXPLORATION")
    print("PRESS '2' TO STOP EXPLORATION AND STOP THE ROBOT")
    print("PRESS 't' TO START TESTING MODE")
    print("PRESS 'q' TO QUIT")

    while not rospy.is_shutdown():
        # Get user input
        choice = input("Enter your choice: ")

        # Handle choices
        if choice == "0":
            control_move_base_pub.publish("start_move_base")
            rospy.loginfo("Published: start_move_base")

            rospy.sleep(3)

            # 创建目标位置的 PoseStamped 消息
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = "map"  # 确保目标点是在 map 坐标系下
            goal_msg.header.stamp = rospy.Time.now()

            # 设置目标点的坐标为 (0, 0, 0) 和默认方向
            goal_msg.pose.position.x = 0.0
            goal_msg.pose.position.y = 0.0
            goal_msg.pose.position.z = 0.0
            goal_msg.pose.orientation.w = 1.0  # 朝向默认方向，无旋转

            # 发布目标点
            goal_pub.publish(goal_msg)
            rospy.loginfo("Published goal to move_base: (0, 0)")
            


        elif choice == "1":
            control_explore_pub.publish("start_explore")
            rospy.loginfo("Published: start_explore")

            control_move_base_pub.publish("start_move_base")
            rospy.loginfo("Published: start_move_base")

        elif choice == "2":
            control_explore_pub.publish("stop_explore")
            rospy.loginfo("Published: stop_explore")

            control_move_base_pub.publish("stop_move_base")
            rospy.loginfo("Published: stop_move_base")
            
            # Send zero velocity to stop the robot
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            velocity_pub.publish(stop_msg)
            rospy.loginfo("Published: stop velocity to halt the robot")
            
            # Send cancel command to move_base
            cancel_msg = GoalID()  # Empty GoalID message cancels all goals
            cancel_pub.publish(cancel_msg)
            rospy.loginfo("Published: cancel navigation goal to stop move_base")

        elif choice.lower() == "q":
            control_move_base_pub.publish("quit")
            control_explore_pub.publish("quit")

            rospy.loginfo("Published: quit")
            rospy.signal_shutdown("User requested shutdown")
            break

        elif choice.lower() == "t":
            control_move_base_pub.publish("start_move_base")


        else:
            print("Invalid choice. Please try again.")




if __name__ == "__main__":
    main()

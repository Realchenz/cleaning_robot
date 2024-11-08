#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def main():
    rospy.init_node("control_node", anonymous=True)
    pub = rospy.Publisher("/control_input", String, queue_size=10)

    print("============= CONTROL PANEL =============")
    print("PRESS '1' TO START EXPLORATION")
    print("PRESS '2' TO STOP EXPLORATION")
    print("PRESS 'q' TO QUIT")

    rospy.loginfo("Control node is running. Press '1' to start exploration, '2' to stop, 'q' to quit.")

    while not rospy.is_shutdown():
        # Get user input
        choice = input("Enter your choice: ")

        # Handle choices
        if choice == "1":
            pub.publish("start_explore")
            rospy.loginfo("Published: start_explore")

        elif choice == "2":
            pub.publish("stop_explore")
            rospy.loginfo("Published: stop_explore")

        elif choice.lower() == "q":
            pub.publish("quit")
            rospy.loginfo("Published: quit")
            rospy.signal_shutdown("User requested shutdown")
            break

        else:
            print("Invalid choice. Please try again.")

if __name__ == "__main__":
    main()

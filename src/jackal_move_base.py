#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

def send_navigation_goal(x, y, yaw):
    rospy.init_node('send_navigation_goal')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = yaw

    client.send_goal(goal)
    client.wait_for_result()

if __name__ == '__main__':
    try:
        send_navigation_goal(10.0, 20.0, 0.0)
    except rospy.ROSInterruptException:
        pass

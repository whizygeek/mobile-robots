#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class ReverseCmdVelNode:
    def __init__(self):
        rospy.init_node('reverse_cmd_vel_node', anonymous=True)

        cmd_vel_topic = '/jackal_velocity_controller/cmd_vel'
        self.cmd_vel_sub = rospy.Subscriber(cmd_vel_topic, Twist, self.cmd_vel_callback)

        reversed_cmd_vel_topic = '/reversed_cmd_vel'
        self.reversed_cmd_vel_pub = rospy.Publisher(reversed_cmd_vel_topic, Twist, queue_size=10)

    def cmd_vel_callback(self, data):
        reversed_cmd_vel = Twist()

        reversed_cmd_vel.linear.x = -data.linear.x
        reversed_cmd_vel.linear.y = -data.linear.y
        reversed_cmd_vel.linear.z = -data.linear.z

        reversed_cmd_vel.angular.x = -data.angular.x
        reversed_cmd_vel.angular.y = -data.angular.y
        reversed_cmd_vel.angular.z = -data.angular.z

        self.reversed_cmd_vel_pub.publish(reversed_cmd_vel)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ReverseCmdVelNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

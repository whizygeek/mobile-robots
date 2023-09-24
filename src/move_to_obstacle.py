#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

MIN_RANGE = 3.4
STOPPING_DISTANCE = 0.5  

Kp = 0.1  
Ki = 0.0  
Kd = 0.0 

desired_distance = 0.5 

error = 0.0
integral = 0.0
derivative = 0.0
prev_error = 0.0

def lidar_callback(data, cmd_vel_pub):
    global error, integral, derivative, prev_error

    min_range_index = data.ranges.index(min(data.ranges))
    min_range_value = data.ranges[min_range_index]
    angle_to_obstacle = min_range_index * data.angle_increment + data.angle_min
    
    cmd_vel_msg = Twist()

    if min_range_value < MIN_RANGE:
        error = desired_distance - min_range_value
        integral += error
        derivative = error - prev_error
        control_signal = Kp * error + Ki * integral + Kd * derivative
        prev_error = error
        cmd_vel_msg.linear.x = 0.2
        cmd_vel_msg.angular.z = control_signal
        
        if min_range_value < STOPPING_DISTANCE:
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
    else:
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0

    cmd_vel_pub.publish(cmd_vel_msg)

def main():
    rospy.init_node('obstacle_detection', anonymous=True)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, lidar_callback, cmd_vel_pub)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32



if __name__ == '__main__':
    rospy.init_node('pub_vel')
    rate = rospy.Rate(10)
    vel_pub = Twist

    while not rospy.is_shutdown():
        vel_pub.linear.x = 1.0
        

        rospy.loginfo("Hello")
        rate.sleep()
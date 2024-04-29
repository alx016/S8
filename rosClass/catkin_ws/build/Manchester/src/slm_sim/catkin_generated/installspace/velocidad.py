#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.init_node('cmd_vel_publisher')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        twist_msg = Twist()
        twist_msg.linear.x = 0.2  # Velocidad lineal en el eje x (m/s)
        twist_msg.angular.z = 0.1  # Velocidad angular en el eje z (rad/s)
        pub.publish(twist_msg)
        rate.sleep()

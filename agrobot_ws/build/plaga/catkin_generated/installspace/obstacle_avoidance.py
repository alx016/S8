#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty

class ObstacleAvoidance:
    def __init__(self):
        self.count1, self.count2 = 0, 0
        self.key = True
        self.msg = Twist()
        self.msg.linear.x = 0
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.x = 0
        self.msg.angular.y = 0
        self.msg.angular.z = 0

        # Initialize ROS node
        rospy.init_node('obstacle_avoidance_node')
        rospy.loginfo("Obstacle Avoidance Node Initialized")

        # Set the rate for the ROS node
        self.rate = rospy.Rate(10)

        # Publisher for velocity commands
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Subscriber for key data
        rospy.Subscriber('/key', Bool, self.key_callback)

        # Subscriber for LiDAR data
        rospy.Subscriber("/scan", LaserScan, self.scan)

        rospy.spin()

    def key_callback(self, data):
        self.key = data

    def stop(self):
        self.msg.linear.x = 0
        self.msg.linear.y = 0
        self.msg.angular.z = 0
        self.pub.publish(self.msg)

    def scan(self, data):
        #rospy.loginfo("Start of scan callback")
        #rospy.loginfo(len(data.ranges))
        #rospy.loginfo(data)

        forward_safe_distance = min(data.ranges[0:70] + data.ranges[650:720])
        right_safe_distance = min(data.ranges[580:650])
        left_safe_distance = min(data.ranges[70:140])
        #rospy.loginfo(forward_safe_distance)

        #rospy.loginfo("right: {}".format(right_safe_distance))

        dist = 0.20
        vel_lin = 0.08
        vel_ang = 0.3
        kp = 0.01

        if self.key == True:
            # rospy.loginfo("ENTRE")
            # rospy.loginfo("forward_safe_distance")
            # rospy.loginfo(forward_safe_distance)
            # rospy.loginfo("right_safe_distance")
            # rospy.loginfo(right_safe_distance)
            if right_safe_distance < dist:
                self.msg.linear.x = 0.01
                self.msg.angular.z = vel_ang- 0.1
                #rospy.loginfo("Turning left")
                self.rate.sleep()
            elif right_safe_distance > dist + 0.13:
                self.msg.linear.x = 0.04
                self.msg.angular.z = -vel_ang
                #rospy.loginfo("Turning right")
                self.rate.sleep()
            elif forward_safe_distance < dist:
                self.msg.linear.x = -0.02
                self.msg.angular.z = vel_ang
                #rospy.loginfo("Izquierda Choque frontal")

            else:
                y_vel = kp * (right_safe_distance - 0.24)
                #y_vel = 1
                # rospy.loginfo(y_vel)
                if y_vel > 0.10:
                    rospy.loginfo("if 1")
                    y_vel = 0.10
                if y_vel < -0.10:
                    rospy.loginfo("if 2")
                    y_vel = -0.10
                self.msg.linear.y = 0
                self.msg.linear.x = vel_lin
                self.msg.angular.z = 0
                #rospy.loginfo("Moving forward")
                self.rate.sleep()
        else:
            self.stop()
            rospy.loginfo("Stop obstacle avoidance")

        self.pub.publish(self.msg)

if __name__ == '__main__':
    try:
        obstacle_avoidance = ObstacleAvoidance()
    except rospy.ROSInterruptException:
        pass

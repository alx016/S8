#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

class plagueMap:
    def __init__(self):

        #Initialize variables


        #Initialize ROS node
        rospy.init_node("plagueMap")
        rospy.loginfo("Plague Map Node Initialized")

        rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        rospy.Subscriber("/map", )
        rospy.Subscriber("/")    #This subscriber will receive information from the detection 

    def odometry_callback(self, msg):
        self.position = [msg.pose.pose.position.x, msg.pose.pose.position.y]

    def processData(self):


    


if __name__ == '__main__':
    try:
        plagueMap = plagueMap()
        while not rospy.is_shutdown():
            plagueMap.processData()

    except rospy.ROSInterruptException:
        pass
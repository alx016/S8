#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion
import tf2_ros


class OdomToBaseLink:
    def __init__(self):
        rospy.init_node('odom_to_base_link')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Subscriber('odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        transform_msg = TransformStamped()
        transform_msg.header.stamp = rospy.Time.now()
        transform_msg.header.frame_id = 'odom'
        transform_msg.child_frame_id = 'base_link'
        transform_msg.transform.translation.x = msg.pose.pose.position.x
        transform_msg.transform.translation.y = msg.pose.pose.position.y
        transform_msg.transform.translation.z = msg.pose.pose.position.z
        transform_msg.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform_msg)

if __name__ == '__main__':
    odom_to_base_link = OdomToBaseLink()
    rospy.spin()

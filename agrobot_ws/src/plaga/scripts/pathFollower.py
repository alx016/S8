#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool # type: ignore
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
import numpy as np
import tf
import logging
from tf.transformations import quaternion_from_euler

class PathFollower:
    def __init__(self):
        rospy.init_node('path_follower', anonymous=True)

        self.velocity_command = Twist()

        # Rate at which to publish velocity commands
        self.rate = rospy.Rate(10) 
        
        self.optimal_path = None            # Variable to store the optimal path
        self.opt_path_receive = False       # Key variable, so that it only receives path once
        self.pathFollower_activationKey = False
        self.actual_pose = [0.0, 0.0, 0.0]  # Variable to store the actual position (x, y, theta)

        # PID Traslational Control constants  
        self.translational_kp = 4.0         # Proportional gain
        self.translational_ki = 0.015       # Integral gain
        self.translational_kd = 1.0         # Derivative gain

        # # PID Rotational Control constants
        self.rotational_kp = 0.8              # Proportional gain
        self.rotational_kd = 0.1

        # Initialize PID controller state
        self.prev_error = [0.0, 0.0]
        self.integral = [0.0, 0.0]

        # Publishers N Subscribers
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/rrt_path', Path, self.path_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/pathFollower_activationKey', Bool, self.activationKey_callback)
        rospy.wait_for_message("/odom", Odometry, timeout = 30)


    def activationKey_callback(self, data):
        self.pathFollower_activationKey = data.data

    def path_callback(self, data):
        for pose_stamped in data.poses:
            pose_stamped.pose.position.x = -pose_stamped.pose.position.x
            pose_stamped.pose.position.y = -pose_stamped.pose.position.y
        self.optimal_path = data

        # print(self.optimal_path)

        if ((not self.opt_path_receive)):
            # Call the function to follow the optimal path
            self.follow_optimal_path()
            self.opt_path_receive = True

    def odom_callback(self, data):
        # Update the current pose of the robot
        
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        self.actual_pose = [data.pose.pose.position.x, data.pose.pose.position.y, yaw] #Obtains the actual position of the bot
        # print(self.actual_pose)

    def follow_optimal_path(self):

        threshold_linear_error = 0.3        #metros
        threshold_rotational_error = 0.5    #radianes
        error = [0.0, 0.0]                  #x, y, theta

        if self.optimal_path is None:
            rospy.logwarn("Optimal path data is missing.")
            
        # Loop through the optimal path and send velocity commands
        i = 0
        for pose_stamped in self.optimal_path.poses:
            xd = pose_stamped.pose.position.x           #desired x
            yd = pose_stamped.pose.position.y           #desired y
            thetad = self.estimate_theta(xd, yd)        #desired theta
            error = self.calculate_error(xd, yd, thetad)
            print (f"{i}")
            # print("xd:", xd)
            # print("yd:", yd)
            # print("td:", thetad)


            while abs(error[0]) >= threshold_linear_error :
                while abs(error[1]) >= threshold_rotational_error:
                    error = self.calculate_error(xd, yd, thetad)
                    self.calculate_rotational_velocity(error)
                    self.velocity_pub.publish(self.velocity_command)
                    print("Rotational Error:", error[1])
                    self.rate.sleep()
                self.velocity_pub.publish(Twist())
                error = self.calculate_error(xd, yd, thetad)
                self.calculate_linear_velocity(error)
                self.velocity_pub.publish(self.velocity_command)
                print("Lineal Error:", error[0])
                self.rate.sleep()
                
            self.velocity_pub.publish(Twist())
            i = i + 1

        # Stop the robot after reaching the end of the path
        print("Llegue")
        self.velocity_pub.publish(Twist())

    def estimate_theta(self, xd, yd):
        # Estimate desired theta
        y = yd - self.actual_pose[1]
        x = xd - self.actual_pose[0]
        thetad = np.arctan2(y, x)
        return thetad

    def calculate_error(self, xd, yd, thetad):
        # Calculate error in the current pose 
        error_x = np.power(xd - self.actual_pose[0], 2)
        error_y = np.power(yd - self.actual_pose[1], 2)
        error_dist = np.sqrt(error_x + error_y)

        error_theta = thetad - self.actual_pose[2] 

        if error_theta > np.pi:
            error_theta -= 2 * np.pi
        elif error_theta < -np.pi:
            error_theta += 2 * np.pi

        error = [error_dist, error_theta]
        return error

    def calculate_linear_velocity(self, error):
        # Pasar del marco inercial al marco del body con la matriz de rotación en z
        # Calculate velocity with PID control

        v_max_linear = 0.1         #maximum linear velocity

        self.integral[0] += error[0] * self.rate.sleep_dur.to_sec()

        self.velocity_command.linear.x = self.translational_kp * error[0] + self.translational_ki * self.integral[0] + self.translational_kd * (error[0] - self.prev_error[0])      
        self.velocity_command.linear.x = v_max_linear * np.tanh(self.velocity_command.linear.x / v_max_linear)
        # print("Velocidad lineal", self.velocity_command.linear.x)

        # Update previous error for the next iteration
        self.prev_error = error

    def calculate_rotational_velocity(self, error):
        v_max_angular = 0.3         # maximum angular velocity

        self.velocity_command.angular.z = self.rotational_kp * error[1] + self.rotational_kd * (error[1] - self.prev_error[1])
        self.velocity_command.angular.z = np.clip(self.velocity_command.angular.z, -v_max_angular, v_max_angular)
        
        # Update previous error for the next iteration
        self.prev_error[1] = error[1]
        # print("Velocidad angular", self.velocity_command.angular.z)

if __name__ == '__main__':
    path_follower = PathFollower()
    rospy.spin()
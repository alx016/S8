#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
import numpy as np

class DifferentialRobot:
    def __init__(self):
        rospy.init_node('differential_robot')
        self.rate = rospy.Rate(10)  # 10 Hz
        self.wheel_base = 0.5  # Distancia entre las llantas (en metros)
        self.left_velocity = 0.0
        self.right_velocity = 0.0
        self.pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.position_pub = rospy.Publisher('robot_position', Odometry, queue_size=10)
        self.time_pub = rospy.Publisher('robot_time', Float64, queue_size=10)  # Nuevo publisher
        self.w_left_pub = rospy.Publisher('w_left', Float64, queue_size=10) 
        self.w_right_pub = rospy.Publisher('w_right', Float64, queue_size=10) 
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        
    def cmd_vel_callback(self, msg):
        # Calcular las velocidades lineales de cada llanta
        self.left_velocity = msg.linear.x - (msg.angular.z * self.wheel_base / 2.0)
        self.right_velocity = msg.linear.x + (msg.angular.z * self.wheel_base / 2.0)
    
    def calculate_odometry(self):
        while not rospy.is_shutdown():
            # Verificar si se ha solicitado el cierre del nodo de ROS
            if rospy.is_shutdown():
                break
            
            # Calcular la velocidad angular de cada llanta
            self.w_left = self.left_velocity / self.wheel_base
            self.w_right = self.right_velocity / self.wheel_base
            
            # Actualizar la pose del robot
            dt = self.rate.sleep_dur.to_sec()  # Tiempo transcurrido entre iteraciones
            self.pose['x'] += (self.left_velocity + self.right_velocity) / 2.0 * np.cos(self.pose['theta']) * dt
            self.pose['y'] += (self.left_velocity + self.right_velocity) / 2.0 * np.sin(self.pose['theta']) * dt
            self.pose['theta'] += (self.w_right - self.w_left) * dt
            
            # Publicar la odometría
            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            odom_msg.pose.pose.position.x = self.pose['x']
            odom_msg.pose.pose.position.y = self.pose['y']
            odom_msg.pose.pose.orientation.z = np.sin(self.pose['theta'] / 2)
            odom_msg.pose.pose.orientation.w = np.cos(self.pose['theta'] / 2)
            self.odom_pub.publish(odom_msg)
            
            # Publicar la posición en el tema personalizado
            position_msg = Odometry()
            position_msg.header.stamp = rospy.Time.now()
            position_msg.header.frame_id = 'odom'
            position_msg.child_frame_id = 'base_link'
            position_msg.pose.pose.position.x = self.pose['x']
            position_msg.pose.pose.position.y = self.pose['y']
            position_msg.pose.pose.orientation.z = np.sin(self.pose['theta'] / 2)
            position_msg.pose.pose.orientation.w = np.cos(self.pose['theta'] / 2)
            self.position_pub.publish(position_msg)

            self.w_left_pub.publish(self.w_left)
            self.w_right_pub.publish(self.w_right)
            
            # Publicar el tiempo
            self.time_pub.publish(rospy.Time.now().to_sec())
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        robot = DifferentialRobot()
        robot.calculate_odometry()
    except rospy.ROSInterruptException:
        pass

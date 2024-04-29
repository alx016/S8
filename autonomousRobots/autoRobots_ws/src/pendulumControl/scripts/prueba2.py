#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64, Time

def wrap_to_pi(theta):
    result = np.fmod((theta + np.pi), (2 * np.pi))  # Wrap angle to [-pi, pi]
    if result < 0:
        result += 2 * np.pi  # Adjust negative results
    return result - np.pi  # Shift back to [-pi, pi]

def position_x_y(theta):
    l = 1  # Longitud del pendulo
    return l * np.sin(theta), -l * np.cos(theta)

def pendulum_simulator():
    rospy.init_node('pendulum_simulator')
    rate = rospy.Rate(10)  # Publish rate of 10 Hz

    theta_pub = rospy.Publisher('pendulum_theta', Float64, queue_size=10)
    omega_pub = rospy.Publisher('pendulum_omega', Float64, queue_size=10)
    time_pub = rospy.Publisher('/time', Time, queue_size=10)

    num_simulations = 5
    max_time = 100
    dt = 0.1
    init_time = rospy.Time.now()
    time = np.arange(0, max_time, dt)
    for sim in range(num_simulations):
        theta1 = np.pi + np.pi/2    #posicion inicial
        theta1 = wrap_to_pi(theta1)
        theta2 = 0      #velocidad
        for i in time:
            theta_pub.publish(theta1)
            omega_pub.publish(theta2)
            time_msg = Time()
            time_msg.data.secs = i
            time_pub.publish(time_msg)

            rospy.loginfo(theta1)
            rospy.loginfo(time_msg)

            theta = np.array([[theta1], [theta2]])

            g = 9.81
            l = 1
            k1 = (g / l) * -10
            k2 = -7

            A = np.array([[0, 1], [-g / l, 0]])
            B = np.array([[0], [1]])
            k = np.array([[k1, k2]])

            omega = np.dot((A + np.dot(B, k)), theta)

            noise = np.random.normal(loc=0, scale=1, size=1)
            theta2 += omega[1][0] * dt + noise[0]
            theta1 += theta2 * dt

            theta1 = wrap_to_pi(theta1)

            rate.sleep()

if __name__ == '__main__':
    try:
        pendulum_simulator()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64, Time, Float64MultiArray

def wrap_to_pi(theta):
    result = np.fmod((theta + np.pi), (2 * np.pi))  # Wrap angle to [-pi, pi]
    if result < 0:
        result += 2 * np.pi  # Adjust negative results
    return result - np.pi  # Shift back to [-pi, pi]

def position_x_y(theta):
    l = 1  # Longitud del pendulo
    return l * np.sin(theta), -l * np.cos(theta)

def omega_cal(theta):
    # Constantes
    g = 9.81
    l = 1
    k1 = (g / l) * -10
    k2 = -7
    A = np.array([[0, 1], [-g / l, 0]])
    B = np.array([[0], [1]])
    k = np.array([[k1, k2]])
    return np.dot((A + np.dot(B, k)), theta)

def pendulum_simulator():

    # Valores iniciales 
    theta1 = np.pi + np.pi/2    #posicion inicial
    theta1 = wrap_to_pi(theta1)
    theta2 = 0                  #velocidad

    num_simulations = 5

    theta1_v = [theta1]*num_simulations
    theta2_v = [theta2]*num_simulations

    rospy.init_node('pendulum_simulator')
    rate = rospy.Rate(100)  # Publish rate of 10 Hz

    theta1_pub = rospy.Publisher('pendulum_theta1', Float64, queue_size=10)
    theta2_pub = rospy.Publisher('pendulum_theta2', Float64, queue_size=10)
    theta3_pub = rospy.Publisher('pendulum_theta3', Float64, queue_size=10)
    theta4_pub = rospy.Publisher('pendulum_theta4', Float64, queue_size=10)
    theta5_pub = rospy.Publisher('pendulum_theta5', Float64, queue_size=10)
    time_pub = rospy.Publisher('/time', Time, queue_size=10)

    rospy.sleep(15)
    max_time = 100
    dt = 0.1
    time = np.arange(0, max_time, dt)

    for t in time:
        time_msg = Time()
        time_msg.data.secs = t
        time_pub.publish(time_msg)

        theta1_pub.publish(theta1_v[0])
        theta2_pub.publish(theta1_v[1])
        theta3_pub.publish(theta1_v[2])
        theta4_pub.publish(theta1_v[3])
        theta5_pub.publish(theta1_v[4])

        # rospy.loginfo(time_msg)

        for k in range(num_simulations):
            theta = np.array([[theta1_v[k]], [theta2_v[k]]])
            omega = omega_cal(theta)

            noise = np.random.normal(loc=0, scale=1, size=1)
            theta2_v[k] += omega[1][0] * dt + noise[0]
            theta1_v[k] += theta2_v[k] * dt

            theta1_v[k] = wrap_to_pi(theta1_v[k])

            rate.sleep()

if __name__ == '__main__':
    try:
        pendulum_simulator()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
import numpy as np

def wrap_to_pi(theta):
    result = np.fmod((theta + np.pi), (2 * np.pi))  # Wrap angle to [-pi, pi]
    if result < 0:
        result += 2 * np.pi  # Adjust negative results
    return result - np.pi  # Shift back to [-pi, pi]

def position_x_y(theta):
    l = 1  # Longitud del pendulo
    return l * np.sin(theta), -l * np.cos(theta)

def pendulum_simulator():
    rospy.init_node('pendulum_simulator', anonymous=True)
    rate = rospy.Rate(1)  # Frecuencia de publicacion de 100 Hz

    position_pub = rospy.Publisher('pendulum_position', Vector3, queue_size=10)
    velocity_pub = rospy.Publisher('pendulum_velocity', Vector3, queue_size=10)

    initial_time = rospy.get_time()
    max_time = 100

    theta1 = np.pi/ 4 + np.pi / 2  # Posicion inicial
    theta2 = 0  # Velocidad inicial
    #and (rospy.get_time() - initial_time) <= max_time
    while not rospy.is_shutdown() :
        # Simulacion del movimiento del pendulo
        position = Vector3()
        position.x, position.y = position_x_y(theta1)

        velocity = Vector3()
        velocity.x = theta1
        velocity.y = theta2

        position_pub.publish(position)
        # rospy.loginfo(position)
        velocity_pub.publish(velocity)
        # rospy.loginfo(velocity)
        # Actualizacion de las variables del pendulo
        g = 9.81
        l = 1
        k1 = (g / l) * -10
        k2 = -7
        dt = 0.01

        A = np.array([[0, 1],
                      [-g / l, 0]])
        B = np.array([[0],
                      [1]])
        k = np.array([[k1, k2]])

        theta = np.array([[theta1],
                          [theta2]])

        omega = np.dot((A + np.dot(B, k)), theta)

        theta1 += omega[0][0] * dt
        theta2 += omega[1][0] * dt

        theta1 = wrap_to_pi(theta1)

        rate.sleep()

if __name__ == '__main__':
    try:
        pendulum_simulator()
    except rospy.ROSInterruptException:
        pass

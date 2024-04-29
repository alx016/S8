#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float64

# Constantes
m1 = 2
m2 = 2
l1 = 1
l2 = 1
g = 9.8

# Punto de equilibrio alrededor del cual linealizar
theta1_eq = 0.0
theta2_eq = 0.0
theta1_dot_eq = 0.0
theta2_dot_eq = 0.0

# Variables de estado iniciales
theta3 = 0  # Theta1 -> Theta3
theta4 = np.pi / 16  # Theta2 -> Theta4
theta3_dot = theta1_dot_eq
theta4_dot = theta2_dot_eq

# Listas para almacenar las posiciones de los péndulos
positions3 = []
positions4 = []

# Función para calcular las derivadas segundas de los ángulos theta
def compute_second_derivatives(theta3, theta4, theta3_dot, theta4_dot):
    theta3_ddot = (-g * (2 * m1 + m2) * theta3 - m2 * g * (theta3 - 2 * theta4) - 2 * (theta3 - theta4) * m2 * (theta4_dot ** 2 * l2 + theta3_dot ** 2 * l1)) / (l1 * (2 * m1 + m2 - m2))
    theta4_ddot = (2 * (theta3 - theta4) * (theta3_dot ** 2 * l1 * (m1 + m2) + g * (m1 + m2) + theta4_dot ** 2 * l2 * m2)) / (l2 * (2 * m1 + m2 - m2))
    return theta3_ddot, theta4_ddot

# Matriz jacobiana
def jacobian_matrix(theta3, theta4, theta3_dot, theta4_dot):
    J = np.zeros((4, 4))
    J[0, 2] = 1
    J[1, 3] = 1
    J[2, 0] = -g
    J[2, 1] = -g
    J[3, 0] = (2 * (theta3 - theta4) * (theta3_dot ** 2 * l1 * (m1 + m2) + g * (m1 + m2) + theta4_dot ** 2 * l2 * m2)) / (l1 * (2 * m1 + m2 - m2))
    J[3, 1] = (-2 * (theta3 - theta4) * (theta3_dot ** 2 * l1 * (m1 + m2) + g * (m1 + m2) + theta4_dot ** 2 * l2 * m2)) / (l2 * (2 * m1 + m2 - m2))
    J[3, 2] = (-2 * (theta3 - theta4) * (theta3_dot ** 2 * l1 * (m1 + m2) + g * (m1 + m2) + theta4_dot ** 2 * l2 * m2)) / (l1 * (2 * m1 + m2 - m2))
    J[3, 3] = (2 * (theta3 - theta4) * (theta3_dot ** 2 * l1 * (m1 + m2) + g * (m1 + m2) + theta4_dot ** 2 * l2 * m2)) / (l2 * (2 * m1 + m2 - m2))
    return J

# Matriz jacobiana evaluada en el punto de equilibrio
Jacobian = jacobian_matrix(theta1_eq, theta2_eq, theta1_dot_eq, theta2_dot_eq)


# Función para simular el movimiento del péndulo
def simulate_pendulum():
    global theta3, theta4, theta3_dot, theta4_dot
    rospy.init_node('pendulum_simulator', anonymous=True)
    rate = rospy.Rate(100)  # Frecuencia de actualización

    # Publicadores para los ángulos
    theta3_pub = rospy.Publisher('theta3', Float64, queue_size=10)
    theta4_pub = rospy.Publisher('theta4', Float64, queue_size=10)

    # Simulación del movimiento del péndulo
    while not rospy.is_shutdown():
        # Calcular las derivadas segundas de los ángulos theta
        theta3_ddot, theta4_ddot = compute_second_derivatives(theta3, theta4, theta3_dot, theta4_dot)

        # Actualizar las velocidades angulares
        theta3_dot += theta3_ddot * 0.001
        theta4_dot += theta4_ddot * 0.001

        # Actualizar los ángulos
        theta3 += theta3_dot * 0.001
        theta4 += theta4_dot * 0.001

        # Publicar los ángulos
        theta3_pub.publish(theta3)
        theta4_pub.publish(theta4)

        # Almacenar las posiciones de los péndulos
        positions3.append(theta3)
        positions4.append(theta4)

        rate.sleep()


if __name__ == '__main__':
    try:
        simulate_pendulum()
    except rospy.ROSInterruptException:
        pass

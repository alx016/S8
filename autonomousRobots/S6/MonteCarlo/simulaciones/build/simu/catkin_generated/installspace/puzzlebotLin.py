#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64
import matplotlib.pyplot as plt


import numpy as np

def publish_trajectory():
    rospy.init_node('trajectory_publisher', anonymous=True)
    rate = rospy.Rate(10)  # Frecuencia de publicación del mensaje, 10 Hz en este ejemplo
    pub_x = rospy.Publisher('/robot_position_x', Float64, queue_size=10)
    pub_y = rospy.Publisher('/robot_position_y', Float64, queue_size=10)
    pub_theta = rospy.Publisher('/robot_orientation_theta', Float64, queue_size=10)
    pub_time = rospy.Publisher('/robot_time', Float64, queue_size=10)

    # Definir las matrices A y B obtenidas de la linealización
    B_eq_numeric = np.array([[1, 0], [0, 0], [0, 1]])

    # Definir el tiempo de simulación
    dt = 0.1  # tiempo de muestreo
    total_time = 10
    timesteps = int(total_time / dt)
    time = np.linspace(0, total_time, timesteps)

    # Condiciones iniciales
    x_0 = 0
    y_0 = 0
    theta_0 = np.pi/4  # Cambiado a pi/4
    initial_state = np.array([x_0, y_0, theta_0])

    # Definir la entrada del sistema (velocidades de las ruedas)
    v_r = 5  # velocidad constante de la rueda derecha
    v_l = 0  # velocidad constante de la rueda izquierda
    states = np.zeros((3, timesteps))
    states[:, 0] = initial_state

    theta_0_values = np.zeros(timesteps)
    theta_0_values[0] = theta_0

    for i in range(1, timesteps):
        input_signal = np.array([v_r, v_l])  # Ajustar la forma de la señal de entrada
        states[:, i] = states[:, i - 1] + dt * (B_eq_numeric @ input_signal)
        theta_0_values[i] = theta_0_values[i - 1]  # Theta_0 es constante en este caso
    x_trajectory = np.cumsum(v_r * dt * np.cos(states[2, :]))
    y_trajectory = np.cumsum(v_r * dt * np.sin(states[2, :]))

    for i in range(timesteps):
        # Publicar la posición x
        pub_x.publish(x_trajectory[i])
        # Publicar la posición y
        pub_y.publish(y_trajectory[i])

        pub_time.publish(time[i])
        # Publicar la orientación theta (constante en este ejemplo)
        pub_theta.publish(theta_0)
        rate.sleep()
        # Publicar la posición x
        rospy.loginfo(x_trajectory[i])
        rospy.loginfo("")

        # Publicar la posición y
        rospy.loginfo(y_trajectory[i])  
        rospy.loginfo(" ")

        # Publicar la orientación theta
        pub_theta.publish(theta_0_values[i])
        rospy.loginfo(theta_0_values[i])  
        rospy.loginfo(" ")
        # rospy.loginfo(timesteps)

        rate.sleep()
    
    x_trajectory2 = np.cumsum(v_r * dt * np.cos(states[2, :]))
    y_trajectory2 = np.cumsum(v_r * dt * np.sin(states[2, :]))

    plt.figure(figsize=(8, 6))
    plt.plot(x_trajectory2, y_trajectory2, label='Trayectoria')
    plt.xlim(-50, 50)
    plt.ylim(-50, 50)
    plt.xlabel('Posición x')
    plt.ylabel('Posición y')
    plt.title('Trayectoria del robot con $\\theta = \\pi/4$')

if __name__ == '__main__':
    try:
        publish_trajectory()
    except rospy.ROSInterruptException:
        pass

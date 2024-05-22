#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray

def trajectory_publisher():
    rospy.init_node('trajectory_publisher', anonymous=True)
    rate = rospy.Rate(100)  # Frecuencia de publicación

    # Crear publicadores para la trayectoria, la orientación y el tiempo
    x_pub = rospy.Publisher('/robot/trajectory/x', Float64, queue_size=10)
    y_pub = rospy.Publisher('/robot/trajectory/y', Float64, queue_size=10)
    theta_pub = rospy.Publisher('/robot/theta', Float64, queue_size=10)
    time_pub = rospy.Publisher('/robot/time', Float64, queue_size=10)

    # Define los parámetros del modelo de planta
    r = 0.5  # Radio de las ruedas
    d = 1.91  # Distancia entre las ruedas
    h = 1  # Parámetro de la planta (asumiendo que h no es nulo)

    # Define otros parámetros
    dt = 0.001  # Paso de tiempo
    total_time = 1000  # Tiempo total de simulación
    q = np.array([[0, 0]]).T  # Posición inicial
    theta = np.pi / 2  # Orientación inicial
    qd = np.array([[50, 50]]).T  # Posición deseada
    Kp = np.eye(2)  # Matriz de ganancia proporcional
    rospy.sleep(5)
    # Bucle de simulación
    for t in np.arange(0, total_time, dt):
        # Cálculo de control y actualización de posición
        D = np.array([[-(d * np.sin(theta) - 2 * h * np.cos(theta)) / (2 * r * h), (d * np.cos(theta) + 2 * h * np.sin(theta)) / (2 * r * h)],
                      [(d * np.sin(theta) + 2 * h * np.cos(theta)) / (2 * r * h), -(d * np.cos(theta) - 2 * h * np.sin(theta)) / (2 * r * h)]])
        D = D.reshape(2, 2)

        q_error = qd - q
        u = np.linalg.inv(D) @ (qd + Kp @ q_error)
        v = np.dot(D, u)
        phit = np.array([[r / d, -r / d]])
        ang = np.dot(phit, u)
        x_pub.publish(q[0]) 
        y_pub.publish(q[1])
        #rospy.loginfo(q[0])
        #rospy.loginfo(q[1])

        theta_msg = Float64(data=theta)
        time_msg = Float64(data=t)
        theta_pub.publish(theta_msg)
        time_pub.publish(time_msg)

        q = q + v * dt
        theta = theta + ang * dt

        # Publicar los valores
        if np.linalg.norm(q - qd) < 0.05:  # Adjust the threshold as needed
            break

        rate.sleep()

if __name__ == '__main__':
    try:
        trajectory_publisher()
    except rospy.ROSInterruptException:
        pass

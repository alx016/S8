import rospy
import numpy as np
from std_msgs.msg import Float64
from rospy.rostime import Duration

x = 0
y = 0
theta = np.pi / 8
vr = 5
vl = 0
l = 10

# Inicializar el nodo de ROS
rospy.init_node('robot_trajectory_publisher')

# Publicadores para los tópicos de x, y y theta
x_pub = rospy.Publisher('/x', Float64, queue_size=10)
y_pub = rospy.Publisher('/y', Float64, queue_size=10)
theta_pub = rospy.Publisher('/theta', Float64, queue_size=10)

# Tiempo inicial
start_time = rospy.Time.now()

# Bucle principal
while not rospy.is_shutdown():
    # Calcular el tiempo transcurrido
    current_time = rospy.Time.now()
    elapsed_time = current_time - start_time
    elapsed_time_sec = elapsed_time.to_sec()  # Convertir a segundos
    
    # Publicar el valor de x
    x_pub.publish(Float64(x))
    
    # Publicar el valor de y
    y_pub.publish(Float64(y))
    
    # Publicar el valor de theta
    theta_pub.publish(Float64(theta))
    
    # Calcular la odometría
    v = (vr + vl) / 2
    w = (vr - vl) / l
    x_dot = v * np.cos(theta)
    y_dot = v * np.sin(theta)
    theta_dot = w
    
    # Actualizar la posición y el ángulo
    x += x_dot
    y += y_dot
    theta += theta_dot
    
    rospy.sleep(0.1)  # Esperar un breve momento antes de la siguiente iteración para reducir el uso de CPU

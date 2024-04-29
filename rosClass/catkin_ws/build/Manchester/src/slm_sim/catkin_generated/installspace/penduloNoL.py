#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float64
import numpy as np
import matplotlib.pyplot as plt

# Constantes
k = 0.01
m1 = 2
m2 = 2
l1 = 1
l2 = 1
g = 9.80

# Setup Variables to be used
theta1 = 0  # np.pi/4
theta2 = np.pi / 16  # np.pi/8
theta1_dot = 0.0
theta2_dot = 0.0
omega1 = 0.0
omega2 = 0.0
omega1_dot = 0.0
omega2_dot = 0.0

# Lists to store data for plotting
positions1 = []
positions2 = []
velocities1 = []
velocities2 = []
times = []

# Solve equations of motion for double pendulum
def solve_equations(dt):
    global theta1, theta2, omega1, omega2, m1, m2, l1, l2, g
    delta = theta2 - theta1
    theta1_dot = omega1
    theta2_dot = omega2

    num1 = -g * (2 * m1 + m2) * np.sin(theta1)
    num2 = -m2 * g * np.sin(theta1 - 2 * theta2)
    num3 = -2 * np.sin(theta1 - theta2) * m2
    num4 = theta2_dot * theta2_dot * l2 + theta1_dot * theta1_dot * l1 * np.cos(theta1 - theta2)
    den = l1 * (2 * m1 + m2 - m2 * np.cos(2 * theta1 - 2 * theta2))
    alpha1 = (num1 + num2 + num3 * num4) / den

    num5 = 2 * np.sin(theta1 - theta2)
    num6 = (theta1_dot * theta1_dot * l1 * (m1 + m2))
    num7 = g * (m1 + m2) * np.cos(theta1)
    num8 = theta2_dot * theta2_dot * l2 * m2 * np.cos(theta1 - theta2)
    den2 = l2 * (2 * m1 + m2 - m2 * np.cos(2 * theta1 - 2 * theta2))
    alpha2 = (num5 * (num6 + num7 + num8)) / den2

    # Update omega1 and omega2
    omega1_dot = alpha1
    omega2_dot = alpha2

    # Update theta1 and theta2
    omega1 += omega1_dot * dt
    omega2 += omega2_dot * dt

    theta1 += theta1_dot * dt
    theta2 += theta2_dot * dt

def pendulum_simulator():
    rospy.init_node('double_pendulum_simulator')

    # Publishers for theta1 and theta2
    theta1_pubN = rospy.Publisher('theta1', Float64, queue_size=10)
    theta2_pubN = rospy.Publisher('theta2', Float64, queue_size=10)

    # Simulation loop
    rate = rospy.Rate(100)  # 100 Hz
    while not rospy.is_shutdown():
        dt = 0.001
        solve_equations(dt)

        # Publish theta1 and theta2
        theta1_pubN.publish(theta1)
        theta2_pubN.publish(theta2)

        # Store data for plotting
        positions1.append(theta1)
        positions2.append(theta2)
        velocities1.append(theta1_dot)
        velocities2.append(theta2_dot)

        rate.sleep()

if __name__ == '__main__':
    print("Simulación del doble péndulo en ejecución...")
    try:
        pendulum_simulator()
    except rospy.ROSInterruptException:
        pass

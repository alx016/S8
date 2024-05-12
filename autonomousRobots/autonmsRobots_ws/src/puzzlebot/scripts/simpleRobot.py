#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Float64MultiArray
import numpy as np


def puzzlebotSim():
    #Parametros del sistema

    rospy.sleep(5)
    r = 5       #radius of the wheel
    d = 19.1    #distance between the wheels 
    h = 10      #parameter of the plant (assuming h is not null)

    pi = np.pi

    tao = 0.001 #time diferential

    x = np.array([ [0, 0, pi/4] ]).T   #Vector de estados (x, y, theta)
    u = np.array([ [1, 1] ]).T  #Velocidades (wr, wl)
    Phi = np.array([ [r/d, -r/d] ])

    m_msg = Float64MultiArray()
    
    while not rospy.is_shutdown():
        D = np.array([ [r/2 * np.cos(x[2][0]) - h * r/ d * np.sin(x[2][0]), r/2 * np.cos(x[2][0]) + h * r/ d * np.sin(x[2][0])],
                    [r/2 * np.sin(x[2][0]) + h * r/ d * np.cos(x[2][0]), r/2 * np.sin(x[2][0]) - h * r/ d * np.cos(x[2][0])] ])

        D_inv = -h * r * r / d * np.array([ [D[1][1], -D[0][1]],
                                            [-D[1][0], D[0][0] ] ])
        
        M = np.array([ D[0],
                    D[1],
                    Phi[0] ])
        rospy.loginfo(M)
        m_msg.data = M.flatten()

        x = x + tao * (M @ u)


        x_pub.publish(x[0][0])
        y_pub.publish(x[1][0])
        theta_pub.publish(x[2][0])
        m_pub.publish(m_msg)
        rospy.loginfo(m_msg)
        rospy.loginfo(f"x = {x[0][0]}")
        rospy.loginfo(f"y = {x[1][0]}")
        rospy.loginfo(f"theta = {x[2][0]}")
        # rospy.sleep(1)


if __name__ == '__main__':
    try:
        rospy.init_node('puzzlebot_simulator')
        rate = rospy.Rate(100)  # Publish rate of 10 Hz
        x_pub = rospy.Publisher('/x', Float64, queue_size=10)
        y_pub = rospy.Publisher('/y', Float64, queue_size=10)
        theta_pub = rospy.Publisher('/theta', Float64, queue_size=10)
        m_pub = rospy.Publisher('/M', Float64MultiArray, queue_size=10)
        puzzlebotSim()
    except rospy.ROSInterruptException:
        pass

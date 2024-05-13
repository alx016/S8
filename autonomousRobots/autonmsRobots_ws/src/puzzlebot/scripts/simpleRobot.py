#!/usr/bin/env python3

import numpy as np
import time

import rospy
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Twist

from Kalman import Kalman
from control import Control


def puzzlebotSim():
    #Parametros del sistema

    rospy.sleep(5)
    r = 5       #radius of the wheel
    d = 19.1    #distance between the wheels 
    h = 10      #parameter of the plant (assuming h is not null)

    pi = np.pi

    tao = 0.001     #time diferential

    x = np.array([ [1, 1, 0] ]).T   #Vector de estados (x, y, theta)
    u = np.array([ [0.1, 0.1] ]).T  #Velocidades (wr, wl)
    qd = np.array([ [100.0, 150.0] ]).T #posicion deseada (x, y)
    Phi = np.array([ [r/d, -r/d] ])

    k = Kalman(tao)
    c = Control(qd)

    x_msg = Twist()
    x_hat_msg = Twist()
    
    for i in np.arange(0.001, 5, 0.001):

        D = np.array([ [r/2 * np.cos(x[2][0]) - h * r/ d * np.sin(x[2][0]), r/2 * np.cos(x[2][0]) + h * r/ d * np.sin(x[2][0])],
                    [r/2 * np.sin(x[2][0]) + h * r/ d * np.cos(x[2][0]), r/2 * np.sin(x[2][0]) - h * r/ d * np.cos(x[2][0])] ])

        D_inv = -h * r * r / d * np.array([ [D[1][1], -D[0][1]],
                                            [-D[1][0], D[0][0] ] ])
        
        M = np.array([  D[0],
                        D[1],
                        Phi[0] ])

        x_hat = k.kalmanCalculation(x, M, u)
        q = np.array([ x[0], x[1]]) #posicion actual
        u, e = c.controlCalculation(q, D_inv)

        noise = np.random.normal(0, 10, 1)
        x = x + tao *(M @ u + noise)

        x_msg.linear.x = x[0][0]
        x_msg.linear.y = x[1][0]
        x_msg.angular.z = x[2][0]

        x_hat_msg.linear.x = x_hat[0][0]
        x_hat_msg.linear.y = x_hat[1][0]
        x_hat_msg.angular.z = x_hat[2][0]

        x_pub.publish(x_msg)
        x_hat_pub.publish(x_hat_msg)
        tao_pub. publish(tao)

        rospy.loginfo("")
        rospy.loginfo(f"x = {x[0][0]}")
        rospy.loginfo(f"y = {x[1][0]}")
        rospy.loginfo(f"theta = {x[2][0]}")
        rospy.loginfo(f"e= {np.mean(e.flatten())}")
        rospy.loginfo("")
        rospy.loginfo(f"x_hat = {x_hat[0][0]}")
        # rospy.sleep(1)

        if rospy.is_shutdown() or np.mean(e.flatten()) < 10:
            break
        
        tao += tao


if __name__ == '__main__':
    try:
        rospy.init_node('puzzlebot_simulator')
        rate = rospy.Rate(100)  # Publish rate of 10 Hz

        x_pub       = rospy.Publisher('/x', Twist, queue_size=10)
        x_hat_pub   = rospy.Publisher('/x_hat', Twist, queue_size=10)
        tao_pub     = rospy.Publisher('/tao', Float64, queue_size=10)

        puzzlebotSim()
        
    except rospy.ROSInterruptException:
        pass

import rospy
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray

def m_callback(msg):
    global M
    M = np.array(msg.data).reshape((3, 2))  # Assuming M is a 3x3 matrix
    # rospy.loginfo("Received M matrix:")
    # rospy.loginfo(M)

def u_callback(msg):
    global u
    u = msg.data

def kalmanCalculation():

    tao = 0.001
    x_hat = np.array([ [0, 0, 0] ]).T

    P = np.array ([ [0, 0, 0], [0, 0, 0], [0, 0, 0] ])

    Q = np.array ([ [1.5, 0, 0], [0, 1.5, 0], [0, 0, 1.5] ])

    R =  0.05 * np.array([ [1, 0, 0], [0, 1, 0], [0, 0, 1] ])

    x_hat = x_hat + tao * (M @ u - P @ np.linalg.inv(R) @ (x_hat - x))
    P = P + tao * (Q - P @ np.linalg.inv(R) @ P)

if __name__ == '__main__':
    try:
        rospy.init_node('/kalman')
        rate = rospy.Rate(100)  # Publish rate of 10 Hz
        
        rospy.Subscriber("/M", Float64MultiArray, m_callback)
        rospy.Subscriber("/u")
        x_hat_pub = rospy.Publisher("/x_hat", Float64)


    except rospy.ROSInterruptException:
        pass

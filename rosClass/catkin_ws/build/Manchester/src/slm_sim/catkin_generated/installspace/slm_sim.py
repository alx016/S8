#!/usr/bin/env python2
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

# Declare the Constants   
#w1 = x2
#w2 = x3
k = 0.01
m = 0.75
l = 0.36 
g = 9.81

# Setup Variables to be used
Tau = 0.0
x1 = 0.0
x2 = 0.0
last_time = 0.0
a = l/2
J = (4/3)*m*a**2

# Declare the process output message
joint_state = JointState()
joint_state.name = ['joint2']

# Define the callback functions 
def tau_callback(msg):
    global Tau
    Tau = msg.data

# Wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if (result < 0):
        result += 2 * np.pi
    return result - np.pi

# Get the time difference
def getDt():
    global last_time
    current_time = rospy.Time.now()
    dt = (current_time - last_time).to_sec()
    last_time = current_time
    return dt

# Get x1 = q and x2 = q_dot
def solveEquations(dt):
    global x1, x2, J, m, a, g, k, Tau
    x1 += x2*dt
    x1 = wrap_to_Pi(x1)
    x2_dot = (1/(J+m*a**2)) * (-m*g*a*np.cos(x1) - k*x2 + Tau)
    x2 += x2_dot*dt

if __name__=='__main__':
    # Initialise and Setup node
    rospy.init_node("SLM_Sim")

    # Configure the Node
    rate = rospy.Rate(rospy.get_param("~node_rate", 100))

    # Setup the Subscribers
    rospy.Subscriber("/tau", Float32, tau_callback)

    # Setup de publishers
    pub = rospy.Publisher("/joint_states", JointState, queue_size = 10)

    print("The SLM sim is Running")
    try:
        while not rospy.is_shutdown():
            if not last_time:
                last_time = rospy.Time.now()
            else:
                dt = getDt()
                solveEquations(dt)

                # Publish the joint states
                joint_state.header.stamp = rospy.Time.now()
                joint_state.position, joint_state.velocity = [x1], [x2]
                pub.publish(joint_state)
            # Wait and repeat
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
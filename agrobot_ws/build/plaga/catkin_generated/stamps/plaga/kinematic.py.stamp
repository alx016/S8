#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import Twist, PoseStamped, Point, Pose, Quaternion

def get_global_params():
    # Reading parameters from the ROS parameter server
    params = {
        'kmodel_rate': rospy.get_param('/kmodel_rate', 50),
        'odom_rate': rospy.get_param('/odom_rate', 50),
        'control_rate': rospy.get_param('/control_rate', 50),
        'rviz_rate': rospy.get_param('/rviz_rate', 50),
        'inertial_frame_name': rospy.get_param('/inertial_frame_name', 'odom'),
        'robot_frame_name': rospy.get_param('/robot_frame_name', 'base_link'),
        'wheel_radius': rospy.get_param('/wheel_radius', 0.05),
        'track_length': rospy.get_param('/track_length', 0.19),
        'commands_topic': rospy.get_param('/commands_topic', 'cmd_vel'),
        'pose_topic': rospy.get_param('/pose_topic', 'pose'),
        'wl_topic': rospy.get_param('/wl_topic', 'wl'),
        'wr_topic': rospy.get_param('/wr_topic', 'wr'),
        'odometry_topic': rospy.get_param('/odometry_topic', 'odom'),
        'starting_state': rospy.get_param('/starting_state', {'x': 0.0, 'y': 0.0, 'theta': 0.0})
    }

    return params

def wrap_to_Pi(theta):
    result = np.fmod(theta + np.pi, 2 * np.pi)

    if isinstance(theta, np.ndarray):
        result[result < 0] += 2 * np.pi
    elif result < 0: 
        result += 2 * np.pi
    return result - np.pi

def stamp(frame_id, pose):
    # Stamp the current pose with current time
    header = Header(frame_id=frame_id, stamp=rospy.Time.now())
    # Create the PoseStamped object with current time and pose
    return PoseStamped(header=header, pose=pose)

def get_dt(last_timestamp):
    current_time = rospy.Time.now()
    dt = (current_time - last_timestamp).to_sec()
    last_timestamp = current_time
    return dt, last_timestamp

def state_gradient(theta, wr, wl, r, l):
    """ Jacobian matrix"""
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)

    J = np.array([[r * cos_theta / 2.0, r * cos_theta / 2.0], 
                  [r * sin_theta / 2.0, r * sin_theta / 2.0], 
                  [r / (2.0 * l), -r / (2.0 * l)]])
    
    return np.dot(J, np.array([wr, wl]))

def rk_integration(dt, state, wr, wl, r, l):
    # Compute RK4 updates
    k1 = state_gradient(state[2], wr, wl, r, l)
    k2 = state_gradient(state[2] + dt * k1[2] / 2.0, wr, wl, r, l)
    k3 = state_gradient(state[2] + dt * k2[2] / 2.0, wr, wl, r, l)
    k4 = state_gradient(state[2] + dt * k3[2], wr, wl, r, l)

    # Update position and orientation using RK4
    return dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0

def cmd_vel_callback(msg, V, w, listening):
    # Controlling V & w
    if listening[0]:
        V[0] = msg.linear.x
        w[0] = msg.angular.z
        listening[0] = False

def simulate(V, w, listening, s, u2w_mat_inv, pose_publisher, w_l_publisher, w_r_publisher, frame_id):
    # Compute dt based on elapsed time
    dt, last_timestamp = get_dt(listening[1])

    wr, wl = np.dot(u2w_mat_inv, np.array([V[0], w[0]]))

    # We capture the system's current state s = [x y theta]
    curr_s = np.array([s.position.x, s.position.y, s.orientation.z])
    
    curr_s = curr_s + rk_integration(dt, curr_s, wr, wl, params['wheel_radius'], params['track_length'])
    
    # Update state
    s = Pose(position=Point(x=curr_s[0], y=curr_s[1], z=0.0), 
             orientation=Quaternion(x=0.0, y=0.0, z=wrap_to_Pi(curr_s[2]), w=1.0))

    # Publish current state
    pose_publisher.publish(stamp(frame_id, s))
    w_l_publisher.publish(wl)
    w_r_publisher.publish(wr)

    # Reset velocities
    V[0] = 0.0
    w[0] = 0.0
    listening[0] = True

if __name__ == '__main__':
    rospy.init_node('puzzlebot_kinematic_model')

    # Get ROS parameters
    params = get_global_params()

    frame_id = params['inertial_frame_name']
    inertial_frame = params['inertial_frame_name']
    x = params['starting_state']['x']
    y = params['starting_state']['y']
    theta = params['starting_state']['theta']
    r = params['wheel_radius']
    l = params['track_length']
    pose_topic = params['pose_topic']
    wl_topic = params['wl_topic']
    wr_topic = params['wr_topic']
    commands_topic = params['commands_topic']
    sim_rate = params['kmodel_rate']

    # Initialize PoseStamped object to keep track of pose (position Z is non-mutable)
    s = Pose(position=Point(x=x, y=y, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=theta, w=1.0))

    # Desired velocities (inertial frame)
    V = [0.0]
    w = [0.0]
    
    # Control to wheel velocities matrix
    u2w_mat_inv = np.linalg.inv(np.array([[r / 2.0, r / 2.0], [r / (2.0 * l), -r / (2.0 * l)]]))
    
    # Subscriber to commands_topic
    rospy.Subscriber(commands_topic, Twist, lambda msg: cmd_vel_callback(msg, V, w, [listening]))

    # Publishers
    pose_publisher = rospy.Publisher(pose_topic, PoseStamped, queue_size=10)
    w_l_publisher = rospy.Publisher(wl_topic, Float32, queue_size=10)
    w_r_publisher = rospy.Publisher(wr_topic, Float32, queue_size=10)
    
    # Timer to mantain simulation dt constant
    listening = [False, rospy.Time.now()]
    rospy.Timer(rospy.Duration(1.0 / sim_rate), lambda _: simulate(V, w, listening, s, u2w_mat_inv, pose_publisher, w_l_publisher, w_r_publisher, frame_id))

    try:
        rospy.loginfo('Kinematic model node running')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

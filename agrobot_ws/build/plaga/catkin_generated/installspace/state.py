#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Transform, PoseWithCovariance, TwistWithCovariance, Pose, Point, Twist


def odom_callback(msg, listening, odom_msg):
    if listening[0]:
        odom_msg[0] = msg
        listening[0] = False

def broadcast_robot_transform(frame_id, robot_body_id, tf_broadcaster, odom_msg):
    # Prepare the transformation object with all required fields
    robot_transform = TransformStamped(
        header=Header(stamp=rospy.Time.now(), frame_id=frame_id),
        child_frame_id=robot_body_id,
        transform=Transform(
            translation=Vector3(x=odom_msg[0].pose.pose.position.x, y=odom_msg[0].pose.pose.position.y, z=odom_msg[0].pose.pose.position.z),
            rotation=odom_msg[0].pose.pose.orientation
        )
    )
    # Broadcast the transformation
    tf_broadcaster.sendTransform(robot_transform)

def publish_sim(dt, odom_msg, frame_id, robot_body_id, tf_broadcaster, joint_state_publisher, joint_names, joint_positions, effort_constant):
    # The odometry message contains:
    # a. Pose: Position and orientation of the robot in inertial frame
    # b. Twist: linear and angular velocities of the robot wheels in the robot frame

    # What we must do with this information:
    # 1. Public dynamic tf based on new position of the robot so that we can always transform relative to the base, inertial frame
    # 2. Transform the values of joints given by the odometry transformed (publish in /joint_states).
    stamp = rospy.Time.now()
    # 1.
    broadcast_robot_transform(frame_id, robot_body_id, tf_broadcaster, odom_msg)

    # 2.
    joints_velocities = np.array([odom_msg[0].twist.twist.angular.x, 
                                  odom_msg[0].twist.twist.angular.y])
    joint_positions[0] = wrap_to_Pi(joint_positions[0] + joints_velocities * dt)
    
    joint_state_publisher.publish(JointState(header=Header(frame_id=robot_body_id, stamp=stamp), 
                                              position=joint_positions[0], 
                                              velocity=joints_velocities, 
                                              name=joint_names, 
                                              effort=effort_constant))

def talk_to_rviz(_, listening, odom_msg, frame_id, robot_body_id, tf_broadcaster, joint_state_publisher, joint_names, joint_positions, effort_constant):
    dt = (odom_msg[0].header.stamp - listening[1]).to_sec()
    listening[1] = odom_msg[0].header.stamp
    publish_sim(dt, odom_msg, frame_id, robot_body_id, tf_broadcaster, joint_state_publisher, joint_names, joint_positions, effort_constant)
    listening[0] = True

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

if __name__ == '__main__':
    rospy.init_node('joint_state_pub')

    # Get Global ROS parameters
    params = get_global_params()

    frame_id = params['inertial_frame_name']
    robot_body_id = params['robot_frame_name']
    joint_names = rospy.get_param('~joint_names', ['leftWheel', 'rightWheel'])
    joint_initial_positions = rospy.get_param('~joint_initial_positions', [0.0, 0.0])
    joint_states_topic = rospy.get_param('~joint_states', '/joint_states')

    tf_broadcaster = tf2_ros.TransformBroadcaster()
    odom_msg = [Odometry(header=Header(frame_id=frame_id), child_frame_id='', pose=PoseWithCovariance(), twist=TwistWithCovariance())]

    # Suscribe to Odom topic
    rospy.Subscriber(params['odometry_topic'], Odometry, lambda msg: odom_callback(msg, listening, odom_msg))

    # Joint State Publisher to send joint states to rviz simulation
    joint_state_publisher = rospy.Publisher(joint_states_topic, JointState, queue_size=10)

    listening = [False, rospy.Time.now()]
    joint_positions = [np.array(joint_initial_positions)]
    effort_constant = [0.0] * len(joint_names)

    rviz_rate = params['rviz_rate']
    rospy.Timer(rospy.Duration(1.0 / rviz_rate), lambda _: talk_to_rviz(_, listening, odom_msg, frame_id, robot_body_id, tf_broadcaster, joint_state_publisher, joint_names, joint_positions, effort_constant))

    try:
        rospy.loginfo('Joint State node running')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

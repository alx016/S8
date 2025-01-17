#!/usr/bin/env python2
import rospy
import message_filters
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import Vector3, Point, Quaternion, Pose, Twist, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry
import numpy as np

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

def system_gradient(theta, dd, dtheta, r, l):
    return np.array([dd * np.cos(theta), dd * np.sin(theta), dtheta])

def rk4_delta(dt, theta, dd, dtheta, r, l):
    k1 = system_gradient(theta, dd, dtheta, r, l)
    k2 = system_gradient(theta + dt*k1[2]/2.0, dd, dtheta, r, l)
    k3 = system_gradient(theta + dt*k2[2]/2.0, dd, dtheta, r, l)
    k4 = system_gradient(theta + dt*k3[2], dd, dtheta, r, l)
    return dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0

def publish_odom(sx, sy, stheta, r, wl, wr, odom_publisher, frame_id, child_frame_id):
    odom_publisher.publish(Odometry(
        header = Header(frame_id = frame_id, stamp = rospy.Time.now()),
        child_frame_id = child_frame_id,
        # Pose in inertial frame (world_frame)
        pose = PoseWithCovariance(
            pose = Pose(
                position = Point(x = sx, y = sy, z = 0.0),
                orientation = Quaternion(x = 0.0, y = 0.0, z = wrap_to_Pi(stheta), w = 1.0)
            ),
            covariance = None
        ),
        # Twist in child frame (puzzlebot)
        twist = TwistWithCovariance(
            twist = Twist(
                linear = Vector3(x = r * wl, y = r * wr, z = 0.0),
                angular = Vector3(x = wl, y = wr, z = 0.0)
            ),
            covariance = None
        )
    ))

def w_callback(wl, wr, listening, decodematrix, odometry_params):
    if listening[0]:
        v, w = np.dot(decodematrix, np.array([wr.data, wl.data]).T).flatten()
        wl_val = wl.data
        wr_val = wr.data
        listening[0] = False
        return v, w, wl_val, wr_val
    return None, None, None, None

def get_dt(last_timestamp):
    current_time = rospy.Time.now()
    dt = (current_time - last_timestamp).to_sec()
    last_timestamp = current_time
    return dt, last_timestamp

def step(dt, sx, sy, stheta, v, w, r, l):
    delta = rk4_delta(dt, stheta, v, w, r, l)
    sx += delta[0]
    sy += delta[1]
    stheta += delta[2]
    return sx, sy, stheta

def localisation():
    rospy.init_node('localisation')

    # Get Global ROS parameters
    params = {
        'control_rate': 50,
        'kmodel_rate': 50,
        'odom_rate': 50,
        'rviz_rate': 50,
        'wheel_radius': 0.05,
        'track_length': 0.19,
        'inertial_frame_name': 'odom',
        'robot_frame_name': 'base_link',
        'commands_topic': 'cmd_vel',
        'pose_topic': 'pose',
        'wl_topic': 'wl',
        'wr_topic': 'wr',
        'odometry_topic': 'odom',
        'starting_state': {
            'x': 0.0,
            'y': 0.0,
            'theta': 0.0
        },
        'joint_state_pub': {
            'joint_states_topic': '/joint_states',
            'joint_names': ['leftWheel', 'rightWheel'],
            'joint_initial_positions': [0.0, 0.0]
        },
        'control': {
            'kp_V': 0.5,
            'kp_w': 0.82,
            'ki_V': 0.008,
            'ki_w': 0.0002,
            'kd_V': 0.05,
            'kd_w': 0.05,
            'd_tolerance': 0.0025,
            'deg_tolerance': 0.005
        }
    }

    frame_id = params['inertial_frame_name']
    child_frame_id = params['robot_frame_name']
    odometry_params = params['starting_state']
    wheel_radius = params['wheel_radius']
    track_length = params['track_length']
    odometry_topic = params['odometry_topic']

    # Robot parameters
    r = wheel_radius
    l = track_length

    # Initial state variables at inertial state
    sx = odometry_params['x']
    sy = odometry_params['y']
    stheta = odometry_params['theta']

    # Velocities (robot frame)
    v = 0.0
    w = 0.0
    wl = 0.0
    wr = 0.0
    
    decodematrix = np.array([[r / 2.0, r / 2.0], [r / (2*l), - r / (2*l)]])

    # Publisher
    odom_publisher = rospy.Publisher('/' + odometry_topic, Odometry, queue_size=10)

    # wl wr Suscriber
    wlsub = message_filters.Subscriber(params['wl_topic'], Float32)
    wrsub = message_filters.Subscriber(params['wr_topic'], Float32)
    # Synchronizer
    ts = message_filters.ApproximateTimeSynchronizer([wlsub, wrsub], queue_size=10, slop=1.0/params['odom_rate'], allow_headerless=True)
    ts.registerCallback(lambda wl, wr: w_callback(wl, wr, [listening], decodematrix, odometry_params))

    # Control simulation / integration rate
    listening = [False]
    last_timestamp = rospy.Time.now()

    def timer_callback(event):
        nonlocal sx, sy, stheta, v, w, wl, wr, listening, last_timestamp
        dt, last_timestamp = get_dt(last_timestamp)
        if listening[0]:
            return
        sx, sy, stheta = step(dt, sx, sy, stheta, v, w, r, l)
        publish_odom(sx, sy, stheta, r, wl, wr, odom_publisher, frame_id, child_frame_id)
        listening[0] = True

    rospy.Timer(rospy.Duration(1.0/params['odom_rate']), timer_callback)

    try:
        rospy.loginfo('Localisation node running')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    localisation()

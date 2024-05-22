import numpy as np
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Path

# Define the plant model parameters
r = 5  # Radius of the wheels
d = 19.1  # Distance between the wheels
h = 10  # Parameter of the plant (assuming h is not null)

# Simulation parameters
dt = 0.001  # Time step
total_time = 10  # Total simulation time

rospy.init_node('trajectory_publisher')
path_pub = rospy.Publisher('/robot_trajectory', Path, queue_size=10)
x_pub = rospy.Publisher('/robot/trajectory/x', Float64, queue_size=10)
y_pub = rospy.Publisher('/robot/trajectory/y', Float64, queue_size=10)
theta_pub = rospy.Publisher('/robot/theta', Float64, queue_size=10)
time_pub = rospy.Publisher('/robot/time', Float64, queue_size=10)

# Simulate motion
rospy.sleep(5)
for sim in range(20):
    q = np.array([[0, 0]]).T  # Initial position
    theta = np.pi/2  # Initial orientation
    qd = np.array([[5, 5]]).T  # Desired position
    Kp = np.eye(2) # Proportional gain matrix
    trajectory = Path()

    noise_scale = 0.5  # Adjust the noise scale as needed
    
    for t in np.arange(0, total_time, dt):
        D = np.array([
            [-(d*np.sin(theta) - 2*h*np.cos(theta)) / (2*r*h), (d*np.cos(theta) + 2*h*np.sin(theta)) / (2*r*h)],
            [(d*np.sin(theta) + 2*h*np.cos(theta)) / (2*r*h), -(d*np.cos(theta) - 2*h*np.sin(theta)) / (2*r*h)]
        ])
        
        q_error = qd - q

        # Generate random noise
        noise = np.random.normal(loc=0, scale=noise_scale, size=(2, 1))

        # Controller
        u = np.dot(np.linalg.inv(D), (qd + Kp @ q_error + noise))

        # Compute velocity
        v = np.dot(D, u)
        phiT = np.array([[r / d, -r / d]])
        ang = np.dot(phiT, u)

        # Update position
        q = q + v * dt
        theta = theta + ang.flatten()[0] * dt

        # Publish data
        x_pub.publish(q[0][0])
        y_pub.publish(q[1][0])
        theta_pub.publish(theta)
        
        time_msg = Float64(data=t)
        time_pub.publish(time_msg)

        rospy.sleep(dt)

rospy.spin()
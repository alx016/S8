import rospy
import numpy as np
import math

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32

x = 1.0
y = 1.0
theta = 0.0

r = 5
l = 19

A = np.array([[r/2, r/2], 
              [r/l, r/l]])

A_inv = np.linalg.inv(A)

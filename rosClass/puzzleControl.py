import numpy as np
import matplotlib.pyplot as plt

# Define the plant model parameters
r = 5  # Radius of the wheels
d = 19.1  # Distance between the wheels
h = 10  # Parameter of the plant (assuming h is not null)

# Define the plant model matrices
def D(theta):
    return np.array([[(r/2 * np.cos(theta)) - (h * r / d * np.sin(theta)), (r/2 * np.cos(theta)) + (h * r / d * np.sin(theta))],
                     [(r/2 * np.sin(theta)) + (h * r / d * np.cos(theta)), (r/2 * np.sin(theta)) - (h * r / d * np.cos(theta))],
                     [r/d , -r/d]])

def phiT():
    return np.array([[r/d , -r/d]])

# Define the proposed controller
def controller(q, theta, qd, Kp):
    # Compute the error in position
    q_error = qd - q

    # Compute the control input using the inverse of D(theta)
    u = np.dot(np.linalg.inv(D(theta)), qd + np.dot(Kp, q_error))

    return u

# Simulation parameters
dt = 0.01  # Time step
total_time = 10  # Total simulation time

# Initial state
q = np.array([0, 0])  # Initial position
theta = np.pi/2  # Initial orientation
qd = np.array([2, 2])  # Desired position
Kp = np.eye(2)
print(Kp)  # Proportional gain matrix

# Arrays to store trajectory
trajectory = [q]

# Simulate motion
# Simulate motion
for t in np.arange(0, total_time, dt):
    # Compute control input
    u = controller(q, theta, qd, Kp)
    
    # Compute velocity
    v = np.dot(D(theta), u)
    
    # Update position
    q = q + v * dt
    
    # Store trajectory
    trajectory.append(q)
    print(theta)
    
    # Check if close to the desired position
    if np.linalg.norm(q - qd) < 0.01:  # Adjust the threshold as needed
        break



# Convert trajectory to numpy array
trajectory = np.array(trajectory)
print (trajectory)

# Plot trajectory
plt.figure(figsize=(8, 6))
plt.plot(trajectory[:, 0], trajectory[:, 1], label='Robot Trajectory')
plt.plot(0, 0, 'bo', label='Initial Position')
plt.plot(qd[0], qd[1], 'ro', label='Desired Position')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Robot Trajectory')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()

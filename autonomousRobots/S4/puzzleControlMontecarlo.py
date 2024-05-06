import numpy as np
import matplotlib.pyplot as plt

# Define the plant model parameters
r = 5       # Radius of the wheels
d = 19.1    # Distance between the wheels
h = 10      # Parameter of the plant (assuming h is not null)

# Define the plant model matrices
def phiT():
    return np.array([ [r/d, -r/d] ])

# Simulation parameters
init_time = 0
final_time = 10  # Total simulation time
dt = 0.01  # Time step
time = np.arange(init_time, final_time, dt)


q = np.array([ [0, 0] ]).T  # Initial position
theta = np.pi/2  # Initial orientation
qd = np.array([ [5, 5] ]).T  # Desired position
Kp = np.eye(2) # Proportional gain matrix
trajectory = [q]
theta_values = []

noise_scale = 0.9 # Adjust the noise scale as needed

for t in time:
    D = np.array([
        [-(d*np.sin(theta) - 2*h*np.cos(theta)) / (2*r*h), (d*np.cos(theta) + 2*h*np.sin(theta)) / (2*r*h)],
        [(d*np.sin(theta) + 2*h*np.cos(theta)) / (2*r*h), -(d*np.cos(theta) - 2*h*np.sin(theta)) / (2*r*h)]
    ])
    
    q_error = qd - q

    # Generate random noise
    noise = np.random.normal(loc=0,scale=noise_scale, size=(2, 1))
    #print (noise)

    # Controller
    u = np.dot(np.linalg.inv(D), (qd + Kp @ q_error + noise))

    # Compute velocity
    v = np.dot(D, u) #+ noise
    #v_ruidp = v + noise
    ang = np.dot(phiT(), u) #+ noise

    # Update position
    q = q + v * dt
    theta = theta + ang.flatten()[0] * dt

    # Store trajectory
    trajectory.append(q)
    theta_values.append(theta)

    # Check if close to the desired position
    if np.linalg.norm(q - qd) < 0.1:  # Adjust the threshold as needed
        break

# Convert trajectory to numpy array
trajectory = np.array(trajectory)
theta_values = np.array(theta_values)

# Plot the trajectories of all simulations
plt.figure(figsize=(8, 6))
plt.plot(trajectory[:, 0], trajectory[:, 1], label=f'Simulation {1}')
plt.plot(0, 0, 'bo', label='Initial Position')
plt.plot(qd[0], qd[1], 'ro', label='Desired Position')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Robot Trajectories with Noise')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()

# Plot the change in theta over time for all simulations
plt.figure(figsize=(8, 6))
plt.plot(np.arange(len(theta_values)), theta_values, label=f'Simulation {1}')
plt.xlabel('Time')
plt.ylabel('Theta')
plt.title('Change in Theta over Time with Noise')
plt.legend()
plt.grid(True)
plt.show()

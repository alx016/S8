import numpy as np
import matplotlib.pyplot as plt
from kalmanFilter import MyKalmanClass
from normal import Normal

# Simulation parameters
init_time = 0       # Initial time
final_time = 10     # Total simulation time
dt = 0.01           # Time step
time = np.arange(init_time, final_time, dt) # Time vector

# Parámetros del sistema
q  = np.array([ [0, 0] ]).T         # Initial position (2 x 1)
qd = np.array([ [10, 10] ]).T       # Desired position (2 x 1)
theta = np.pi/2                     # Initial orientation

noise_scale = 3 # Adjust the noise scale as needed

n = Normal()
n.init(q, qd, theta, dt)

k = MyKalmanClass()
k.init(q, qd, theta, dt)

for t in time:
    # Generate random noise
    noise = np.random.normal(loc=0,scale=noise_scale, size=(2, 1))  # media, covarianza,  (2 x 1)

    n.calculation(noise)
    k.calculation(noise)

    if n.returnBreakKey() or k.returnBreakKey():
        break


# Convert trajectory to numpy array
trajectory = n.returnTrajectory()
trajectory = np.array(trajectory)
theta_values = n.returnThetaVals()
trajectory_kalman = k.returnTrajectory()
theta_values_kalman = k.returnThetaVals()
theta_values_kalman = np.array(theta_values_kalman)

# Plot the trajectories of all simulations
plt.figure(figsize=(8, 6))
plt.plot(trajectory[:, 0], trajectory[:, 1], label=f'normal')
for i in range(len(trajectory_kalman) - 1):
    plt.plot([trajectory_kalman[i][0], trajectory_kalman[i+1][0]] , [trajectory_kalman[i][1], trajectory_kalman[i+1][1]], label=f'Kalman', color = 'r')  # Plot x and y coordinates together
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
plt.plot(np.arange(len(theta_values)), theta_values, label='normal')
plt.plot(np.arange(len(theta_values_kalman)), theta_values_kalman, label='Kalman')
plt.xlabel('Time')
plt.ylabel('Theta')
plt.title('Change in Theta over Time with Noise')
plt.legend()
plt.grid(True)
plt.show()

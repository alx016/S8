import numpy as np
import matplotlib.pyplot as plt

# Robot Parameters
r = 5       # Radius of the wheels
d = 19.1    # Distance between the wheels
h = 10      # Parameter of the plant (assuming h is not null)

# Define the plant model matrices
def phiT():
    return np.array([ [r/d, -r/d] ])

# Simulation parameters
init_time = 0       # Initial time
final_time = 10     # Total simulation time
dt = 0.01           # Time step
time = np.arange(init_time, final_time, dt) # Time vector

# Parámetros del sistema
q  = np.array([ [0, 0] ]).T     # Initial position (2 x 1)
qd = np.array([ [5, 10] ]).T     # Desired position (2 x 1)
theta = np.pi/2                 # Initial orientation

Kp = np.eye(2)                  # Proportional gain matrix (2 x 2)
trajectory = [q]
theta_values = []

noise_scale = 0.9 # Adjust the noise scale as needed


A = np.array([[1, 0, dt, 0, 0],
              [0, 1, 0, 0, 0],
              [0, 0, 1, 0, 0],
              [0, 0, 0, 1, 0],
              [0, 0, 0, 0, 1]])

H = np.array([[1, 0, 0, 0, 0],
              [0, 1, 0, 0, 0]])

Q = np.array([[0.001, 0, 0, 0.01, 0.01],
              [0, 0.001, 0, 0.01, 0.01],
              [0, 0, 0.001, 0, 0],
              [0.01, 0.01, 0, 0.1, 0.1],
              [0.01, 0.01, 0, 0.1, 0.1]])

R = np.array([[0.01, 0],
              [0, 0.01]])


# Matriz de covarianza inicial
P_est = np.eye(5)
I = np.eye(5)



for t in time:
    D = np.array([
        [-(d*np.sin(theta) - 2*h*np.cos(theta)) / (2*r*h), (d*np.cos(theta) + 2*h*np.sin(theta)) / (2*r*h)],
        [(d*np.sin(theta) + 2*h*np.cos(theta)) / (2*r*h), -(d*np.cos(theta) - 2*h*np.sin(theta)) / (2*r*h)]
    ]) # (2 x 2)

    q_error = qd - q    # (2 x 1)

    # Generate random noise
    noise = np.random.normal(loc=0,scale=noise_scale, size=(2, 1))  # media, covarianza (2 x 1)

    # Controller
    #           (2 x 2 * ((2 x 1) + (2 x 2) * (2 x 1) + (2 x 1))) = (2 x 1)
    u = np.dot(np.linalg.inv(D), (qd + np.dot(Kp, q_error) + noise))

    x_est = np.array([[q[0][0], q[1][0], theta, u[0][0], u[1][0]]]) # state vector (5 x 1)
    u_est = x_est[0][3:5]  # Extract estimated control signal from state vector
    
    # Prediction step
    x_pre = A @ x_est.T     # (5, 5) * (5, 1) = (5, 1)
    P_pre = A @ P_est @ A.T + Q


    # Measurement (assuming no direct measurement of u)
    z = np.array([[q[0][0]], [q[1][0]]])  # Only position measurements

    # Update step
    K = P_pre @ H.T @ np.linalg.inv((H @ P_pre @ H.T + R))  # 5 x 2

    x_est = x_pre.T + K @ (z - H @ x_pre)     # 5 x 5
    P_est = (I - K @ H) * P_pre 

    # Compute velocity
    # v = np.dot(D, u)        # (2 x 2) * (2 x 1) = (2 x 1)
    # ang = np.dot(phiT(), u) # (1 x 2) * (2 x 1) = (1 x 1)
    # Controller
    v = np.dot(D, u_est)        # (2 x 2) * (2 x 1) = (2 x 1)
    ang = np.dot(phiT(), u_est) # (1 x 2) * (2 x 1) = (1 x 1)

    # Update position
    q = q + v * dt                  # (2 x 1) + (2 x 1) * c
    theta = theta + ang[0] * dt  # c + c * c

    # Store trajectory
    # print(q[0][0], q[0][1])
    trajectory.append(np.array([q[0][0], q[0][1]]))
    theta_values.append(theta)

    # Check if close to the desired position
    if np.linalg.norm(q - qd) < 0.1:  # Adjust the threshold as needed
        break

# Convert trajectory to numpy array
# trajectory = np.array(trajectory)
theta_values = np.array(theta_values)

# Plot the trajectories of all simulations
plt.figure(figsize=(8, 6))
for i in range(len(trajectory)):
    plt.plot(trajectory[i], trajectory[i])  # Plot x and y coordinates together
    # plt.scatter(trajectory[i][0], trajectory[i][1])
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

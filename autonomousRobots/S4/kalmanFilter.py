import numpy as np
class MyKalmanClass:
    def init(self, q, qd, theta, dt):
        # Parámetros del sistema
        self.q  = q         # Initial position (2 x 1)
        self.qd = qd        # Desired position (2 x 1)
        self.theta = theta  # Initial orientation
        self.dt = dt

        self.break_key = False

        # Robot Parameters
        self.r = 5       # Radius of the wheels
        self.d = 19.1    # Distance between the wheels
        self.h = 10      # Parameter of the plant (assuming h is not null)

        self.A = np.array([[1, 0, self.dt, 0, 0],
                    [0, 1, 0, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 1]])

        self.H = np.array([[1, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0]])

        self.Q = np.array([[0.001, 0, 0, 0.01, 0.01],
                            [0, 0.001, 0, 0.01, 0.01],
                            [0, 0, 0.001, 0, 0],
                            [0.01, 0.01, 0, 0.1, 0.1],
                            [0.01, 0.01, 0, 0.1, 0.1]])

        self.R = np.array([[0.01, 0],
                        [0, 0.01]])
        
        # Matriz de covarianza inicial
        self.P_est = np.eye(5)
        self.I = np.eye(5)
        
        self.Kp = np.eye(2)                  # Proportional gain matrix (2 x 2)
        self.trajectory = [self.q.T.flatten()]
        self.theta_values = []

    # Define the plant model matrices
    def phiT(self):
        return np.array([ [self.r/self.d, -self.r/self.d] ])

    def returnTrajectory(self):
        return self.trajectory

    def returnThetaVals(self):
        return self.theta_values
    
    def returnBreakKey(self):
        return self.break_key


    def calculation(self, noise):
        D = np.array([
            [-(self.d*np.sin(self.theta) - 2*self.h*np.cos(self.theta)) / (2*self.r*self.h), (self.d*np.cos(self.theta) + 2*self.h*np.sin(self.theta)) / (2*self.r*self.h)],
            [(self.d*np.sin(self.theta) + 2*self.h*np.cos(self.theta)) / (2*self.r*self.h), -(self.d*np.cos(self.theta) - 2*self.h*np.sin(self.theta)) / (2*self.r*self.h)]
        ]) # (2 x 2)

        q_error = self.qd - self.q    # (2 x 1)

        # Controller
        #           (2 x 2 * ((2 x 1) + (2 x 2) * (2 x 1) + (2 x 1))) = (2 x 1)
        u = np.dot(np.linalg.inv(D), (self.qd + np.dot(self.Kp, q_error) + noise))

        x_est = np.array([[self.q[0][0], self.q[1][0], self.theta, u[0][0], u[1][0]]]) # state vector (5 x 1)
        u_est = x_est[0][3:5]  # Extract estimated control signal from state vector
        
        # Prediction step
        x_pre = self.A @ x_est.T     # (5, 5) * (5, 1) = (5, 1)
        P_pre = self.A @ self.P_est @ self.A.T + self.Q

        # Measurement (assuming no direct measurement of u)
        z = np.array([[self.q[0][0]], [self.q[1][0]]])  # Only position measurements

        # Update step
        K = P_pre @ self.H.T @ np.linalg.inv((self.H @ P_pre @ self.H.T + self.R))  # 5 x 2

        x_est = x_pre.T + K @ (z - self.H @ x_pre)     # 5 x 5
        P_est = (self.I - K @ self.H) * P_pre 

        # Compute velocity
        # v = np.dot(D, u)        # (2 x 2) * (2 x 1) = (2 x 1)
        # ang = np.dot(phiT(), u) # (1 x 2) * (2 x 1) = (1 x 1)
        # Controller
        v = np.dot(D, u_est)        # (2 x 2) * (2 x 1) = (2 x 1)
        ang = np.dot(self.phiT(), u_est) # (1 x 2) * (2 x 1) = (1 x 1)

        # Update position
        self.q = self.q + v * self.dt                  # (2 x 1) + (2 x 1) * c
        self.theta = self.theta + ang[0] * self.dt  # c + c * c

        # Store trajectory
        # print(q[0][0], q[0][1])
        self.trajectory.append(np.array([self.q[0][0], self.q[0][1]]))
        self.theta_values.append(self.theta)

            # Check if close to the desired position
        if np.linalg.norm(self.q - self.qd) < 0.1:  # Adjust the threshold as needed
            self.break_key = True
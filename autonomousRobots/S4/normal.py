import numpy as np

class Normal():
    def init(self, q, qd, theta, dt):

        #Initial Parameters
        self.q = q
        self.qd = qd
        self.theta = theta
        self.dt = dt

        # Robot Parameters
        self.r = 5       # Radius of the wheels
        self.d = 19.1    # Distance between the wheels
        self.h = 10      # Parameter of the plant (assuming h is not null)
        
        self.Kp = np.eye(2) 
        self.trajectory = [self.q]
        self.theta_values = []

        self.break_key = False

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
        
        D = D.reshape(2, 2)
        u = np.dot(np.linalg.inv(D), (self.qd + np.dot(self.Kp, q_error) + noise))

        # Calculate
        v = np.dot(D, u)        # (2 x 2) * (2 x 1) = (2 x 1)
        ang = np.dot(self.phiT(), u) # (1 x 2) * (2 x 1) = (1 x 1)

        # Update position
        self.q = self.q + v * self.dt                  # (2 x 1) + (2 x 1) * c
        self.theta = self.theta + ang[0] * self.dt  # c + c * c

        # Store trajectory
        # print(q[0][0], q[0][1])
        self.trajectory.append(self.q)
        self.theta_values.append(self.theta)

        if np.linalg.norm(self.q - self.qd) < 0.1:  # Adjust the threshold as needed
            self.break_key = True
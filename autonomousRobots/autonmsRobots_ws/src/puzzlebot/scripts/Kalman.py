import numpy as np

class Kalman():
    def __init__(self, tao) -> None:
        self.x_hat = np.array([ [0, 0, 0] ]).T
        self.tao = tao
        self.P = np.array ([ [0, 0, 0], [0, 0, 0], [0, 0, 0] ])

        self.Q = 800 * np.array ([ [1, 0, 0], [0, 1, 0], [0, 0, 1] ]) #noise covariance matrix (covarianza del sistema)

        self.R =  0.000384 * np.array([ [1, 0, 0], [0, 1, 0], [0, 0, 1] ])#measurement noise covariance matrix (matriz de covarianza del ruido de mediciones)

    def kalmanCalculation(self, x, M, u):

        self.x_hat = self.x_hat + self.tao * (M @ u - self.P @ np.linalg.inv(self.R) @ (self.x_hat - x))
        self.P = self.P + self.tao * (self.Q - self.P @ np.linalg.inv(self.R) @ self.P)                     #covarianza del modelo

        return self.x_hat
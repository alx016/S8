import numpy as np

class Kalman:
    def __init__(self, tao):
        # Inicializar las matrices del filtro de Kalman
        self.tao = tao
        self.A = np.eye(3)  # Matriz de transición de estados
        self.B = np.zeros((3, 2))  # Matriz de control
        self.H = np.eye(3)  # Matriz de observación
        self.Q = np.eye(3) * 0.1  # Covarianza del ruido de proceso
        self.R = np.eye(3) * 0.1  # Covarianza del ruido de medida
        self.P = np.eye(3)  # Matriz de covarianza del error

    def predict(self, x, u):
        # Predicción del estado y de la covarianza del error
        self.B = np.array([
            [self.tao * np.cos(x[2]), 0],
            [self.tao * np.sin(x[2]), 0],
            [0, self.tao]
        ])
        x_pred = self.A @ x + self.B @ u
        P_pred = self.A @ self.P @ self.A.T + self.Q
        return x_pred, P_pred

    def update(self, x_pred, P_pred, z):
        # Actualización del estado y de la covarianza del error
        y = z - self.H @ x_pred
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ np.linalg.inv(S)
        x_upd = x_pred + K @ y
        P_upd = (np.eye(3) - K @ self.H) @ P_pred
        return x_upd, P_upd

    def kalmanCalculation(self, x, u, z):
        # Cálculo del filtro de Kalman
        x_pred, P_pred = self.predict(x, u)
        x_upd, P_upd = self.update(x_pred, P_pred, z)
        self.P = P_upd
        return x_upd

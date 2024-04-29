import numpy as np
import matplotlib.pyplot as plt


def wrap_to_pi(theta):
    result = np.fmod((theta + np.pi), (2 * np.pi))  # Wrap angle to [-pi, pi]
    if result < 0:
        result += 2 * np.pi  # Adjust negative results
    return result - np.pi  # Shift back to [-pi, pi]

def position_x_y(theta):
    return l*np.sin((theta)), -l*np.cos((theta))

g = 9.81
l = 1

k1 = -1
k2 = -1

initial_time = 0
max_time = 100
dt = 0.01
t = np.arange(initial_time, max_time, dt)

theta1 = np.pi*8/4 + np.pi/2#posición inicial
theta1 = wrap_to_pi(theta1)
theta2 = 0      #velocidad

A = np.array([[0   , 1],
            [(-g/l)*np.sin(theta1), 0]])
B = np.array([[0],
            [1]])
k = np.array([[k1, k2]])

x, y = position_x_y(theta1)
x_val = [x]
y_val = [y]
theta_val = [theta1]
omega_val = [theta2]

for i in t:
    A = np.array([[0   , 1],
            [(-g/l)*np.cos(theta1), 0]])
    
    theta = np.array([[theta1],   #posicion 
                    [theta2]])  #velocidad

    omega = np.dot((A + np.dot(B, k)), theta)   #2x1  omega = (thetap1, thetap2)  = (velocidad, aceleración)

    theta1 += omega[0][0] * dt  #posicion += velocidad * dt
    theta2 += omega[1][0] * dt  #velocidad += aceleracion * dt

    theta1 = wrap_to_pi(theta1)
    x, y = position_x_y(theta1)

    x_val.append(x)
    y_val.append(y)
    theta_val.append(theta1)
    omega_val.append(theta2)

plt.figure()
plt.scatter(0,0, color='red', label='center')
plt.scatter(x_val, y_val, label='Masa 1')
plt.title('Trayectoria de la masa en un péndulo simple')
plt.xlabel('Posición en x')
plt.ylabel('Posición en y')
plt.xlim(-1, 1)
plt.ylim(-1, 1)
plt.legend()
plt.grid(True)
plt.savefig('pendulum_pos_x_y.png')

plt.figure()
plt.scatter(theta_val, omega_val, label='Masa 1')
plt.title('Posición v velocidad')
plt.xlabel('theta')
plt.ylabel('omega')
plt.legend()
plt.grid(True)
plt.savefig('pendulum_theta_v_omega.png')
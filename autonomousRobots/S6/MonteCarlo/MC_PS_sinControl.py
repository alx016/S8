import numpy as np
import matplotlib.pyplot as plt


def wrap_to_pi(theta):
    result = np.fmod((theta + np.pi), (2 * np.pi))  # Wrap angle to [-pi, pi]
    if result < 0:
        result += 2 * np.pi  # Adjust negative results
    return result - np.pi  # Shift back to [-pi, pi]

def position_x_y(theta):
    return l*np.sin((theta)), -l*np.cos((theta))

# Definir el número de pasos de tiempo y simulaciones Monte Carlo
num_simulaciones = 5

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

for sim in range(num_simulaciones):
    g = 9.81
    l = 1

    k1 = -1
    k2 = -1

    initial_time = 0
    max_time = 100
    dt = 0.1

    t = np.arange(initial_time, max_time, dt)

    A = np.array([[0   , 1],
                [(-g/l), 0]])
    B = np.array([[0],
                [1]])
    k = np.array([[k1, k2]])
    
    theta1 = np.pi + np.pi/2#posición inicial
    theta1 = wrap_to_pi(theta1)
    theta2 = 0      #velocidad

    x, y = position_x_y(theta1)
    x_val = [x]
    y_val = [y]
    theta_val = []
    omega_val = []

    for i in t:
        theta_val.append(theta1)
        omega_val.append(theta2)
        theta = np.array([[theta1],   #posicion 
                        [theta2]])  #velocidad

        omega = np.dot((A + np.dot(B, k)), theta)   #2x1  omega = (thetap1, thetap2)  = (velocidad, aceleración)
        
        noise = np.random.normal(loc=0, scale=0.1, size=1)
        theta2 += omega[1][0] * dt + noise[0] #velocidad += aceleracion * dt
        theta1 += theta2 * dt   #posicion += velocidad * dt

        theta1 = wrap_to_pi(theta1)
        x, y = position_x_y(theta1)
        x_val.append(x)
        y_val.append(y)

    # Graficar la trayectoria de la simulación actual en el primer subgráfico
    ax1.plot(t, theta_val, label=f"Simulación {sim+1}")
    ax1.set_title('Theta vs Tiempo')
    ax1.set_xlabel('Tiempo')
    ax1.set_ylabel('Theta')
    ax1.legend()
    ax1.grid(True)

    # Graficar theta vs omega en el segundo subgráfico
    ax2.plot(t, omega_val, label=f"Simulación {sim+1}")
    ax2.set_title('Velocidad vs Tiempo')
    ax2.set_xlabel('Tiempo')
    ax2.set_ylabel('Omega')
    ax2.legend()
    ax2.grid(True)

plt.tight_layout()  # Ajusta automáticamente el espaciado entre los subgráficos para que no se superpongan
plt.savefig('MC_PS_SC.png')
import numpy as np
import matplotlib.pyplot as plt

# Posición Inicial
x = 0
y = 0
theta = np.pi/4

x_values = [x]
y_values = [y]
theta_values = [theta]

# Velocidad de llantas
v_r = 1
v_l = 1
l = 1

t = list(range(0, 1000, 1))

def eqSolution():
    global x, y, l, theta
    global x_values, y_values, theta_values
    v = (v_r + v_l) / 2
    w = (v_r - v_l)/l

    for i in range(len(t) - 1):
        x += v * np.cos(theta)
        y += v * np.sin(theta)
        w = (v_r - v_l)/l
        theta += w
        x_values.append(x)
        y_values.append(y)
        theta_values.append(theta)

    # Crea una figura y ejes
    fig, (ax1, ax2) = plt.subplots(2, 1)
    ax1.scatter(x_values, y_values, color='b')
    ax1.set_xlim(0, 25)
    ax1.set_ylim(0, 25)
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
    ax1.set_title('x vs y')
    
    ax2.scatter(t, theta_values, color='b')
    ax2.set_xlabel('x')
    ax2.set_ylabel('y')
    ax2.set_title('Time vs Theta')
    fig.savefig('trajectory_plot1.png')

if __name__ == "__main__":
    eqSolution()

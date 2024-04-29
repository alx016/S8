import numpy as np
import matplotlib.pyplot as plt

# Definir el número de pasos de tiempo
num_pasos = 100

# Definir el número de simulaciones
num_simulaciones = 5

# Generar las simulaciones
for i in range(num_simulaciones):
    # Generar la secuencia de ruido blanco w(t)
    # loc -> media , scale -> desviación estandar
    w = np.random.normal(loc=0, scale=1, size=num_pasos)
    
    # Calcular la trayectoria de x_punto
    x_punto = np.cumsum(w)
    
    # Plotear la simulación actual
    plt.plot(range(num_pasos), x_punto, label=f'Simulación {i+1}')

# Configurar el gráfico
plt.xlabel('Tiempo')
plt.ylabel('x_punto')
plt.title('Simulaciones de Monte Carlo')
plt.legend()
plt.grid(True)

# Mostrar el gráfico
plt.savefig('monteCarlo.png')


import numpy as np 
import pandas as pd
import matplotlib.pyplot as plt 

#Parametros del sistema
r = 5       #radius of the wheel
d = 19.1    #distance between the wheels 
h = 10      #parameter of the plant (assuming h is not null)

pi = np.pi

tao = 0.001 #time diferential

x = np.array([ [0, 0, pi/4] ]).T   #Vector de estados (x, y, theta)
u = np.array([ [0.1, 0.1] ]).T  #Velocidades (wr, wl)

Phi = np.array([ [r/d, -r/d] ])

out = x.T

for i in np.arange(0.001, 10, tao):
    D = np.array([ [r/2 * np.cos(x[2][0]) - h * r/ d * np.sin(x[2][0]), r/2 * np.cos(x[2][0]) + h * r/ d * np.sin(x[2][0])],
                   [r/2 * np.sin(x[2][0]) + h * r/ d * np.cos(x[2][0]), r/2 * np.sin(x[2][0]) - h * r/ d * np.cos(x[2][0])] ])

    D_inv = -h * r * r / d * np.array([ [D[1][1], -D[0][1]],
                                        [-D[1][0], D[0][0] ] ])
    
    M = np.array([ D[0],
                  D[1],
                   Phi[0] ])

    x = x + tao * (M @ u)
    print(x[1][0])
    out = np.concatenate((out, x.T), axis=0)
out_df = pd.DataFrame(out, columns=["x", "y", "theta"])

out_df.plot(x="x", y="y")
plt.show()
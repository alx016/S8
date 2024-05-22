import numpy as np 
import pandas as pd
import matplotlib.pyplot as plt 

#Parametros del sistema
r = 5       #radius of the wheel
d = 19.1    #distance between the wheels 
h = 10      #parameter of the plant (assuming h is not null)

pi = np.pi

tao = 0.001 #time diferential

x = np.array([ [1, 1, 0] ]).T   #vector de estados (x, y, theta)
u = np.array([ [0.1, 0.1] ]).T  #velocidades (wr, wl)

qd = np.array([ [100.0, 150.0] ]).T #posicion deseada (x, y)

Phi = np.array([ [r/d, -r/d] ])

x_hat = np.array([ [0, 0, 0] ]).T

P = np.array ([ [0, 0, 0], [0, 0, 0], [0, 0, 0] ])

Q = np.array ([ [1.5, 0, 0], [0, 1.5, 0], [0, 0, 1.5] ])

R =  0.05 * np.array([ [1, 0, 0], [0, 1, 0], [0, 0, 1] ])

dummy = np.concatenate( (np.array([ [0] ]),x.T), axis=1 )
out = np.concatenate( (dummy, x_hat.T), axis=1 )

for i in np.arange(0.001, 10, tao):
    D = np.array([ [r/2 * np.cos(x[2][0]) - h * r/ d * np.sin(x[2][0]), r/2 * np.cos(x[2][0]) + h * r/ d * np.sin(x[2][0])],
                   [r/2 * np.sin(x[2][0]) + h * r/ d * np.cos(x[2][0]), r/2 * np.sin(x[2][0]) - h * r/ d * np.cos(x[2][0])] ])

    D_inv = -h * r * r / d * np.array([ [D[1][1], -D[0][1]],
                                        [-D[1][0], D[0][0] ] ])
    
    M = np.array([ D[0],
                   D[1],
                   Phi[0] ])

    #KALMAN FILTER 
    x_hat = x_hat + tao * (M @ u - P @ np.linalg.inv(R) @ (x_hat - x))
    
    P = P + tao * (Q - P @ np.linalg.inv(R) @ P)

    q = np.array([ x[0], x[1]]) #posicion actual

    e = qd -q 

    # u = D_inv @ (0.1 * e)

    # Physical robot simulation 

    x = x + tao * (M @ u + np.random.normal(0, 10, 1) )
    # x = x + tao * (M @ u )

    dummy = np.concatenate( (np.array([ [i] ]), x.T), axis=1 )
    dummy = np.concatenate( (dummy, x_hat.T), axis=1 )
    out = np.concatenate( (out, dummy), axis=0 )

out_df = pd.DataFrame(out, columns=["t", "x", "y", "theta", "xh", "yh", "th"])

out_df.plot(x="x", y="y")
# out_df.plot(x="t", y="theta")
out_df.plot(x="t", y=["x", "xh"])

plt.show()

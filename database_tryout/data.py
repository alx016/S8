import numpy as np
import time

init = 1
max = 1000
step = 0.1


for i in np.arange(init, max, step):
    var = np.random.randint(15, 50)
    print(var)
    time.sleep(2)
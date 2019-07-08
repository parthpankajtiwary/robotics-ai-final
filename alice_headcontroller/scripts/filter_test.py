import numpy as np
from matplotlib import pyplot as plt


a = 0.1
b = 0.5

small_step = 0.7
step = (1.0 - small_step)

data = []
for i in range(20):
    a = a * step + b * small_step
    data.append(a)
    
plt.plot(data)
plt.show()
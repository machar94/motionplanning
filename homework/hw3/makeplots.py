import matplotlib.pyplot as plt
import numpy as np
import os
import IPython

plt.rcParams.update({'font.size': 22})

# Read data from file
FILENAME = 'goalBias.txt'

data = np.loadtxt(FILENAME, delimiter=',')

plt.plot(data[0,0:], data[1,0:], label='Time')
# plt.plot(bias, samples, label='Samples')
plt.xlabel('Goal Bias')
plt.ylabel('Computation Time (s)')
plt.title('Computation Time vs Goal Bias')

plt.legend()
axes = plt.gca()

# Show the plot
plt.show()
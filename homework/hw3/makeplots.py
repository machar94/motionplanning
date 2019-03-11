import matplotlib.pyplot as plt
import numpy as np
import os
import IPython

plt.rcParams.update({'font.size': 22})

# Read data from file
# FILENAME = 'compTimevsGoal.txt'
FILENAME = 'compTimevsGoalStep05.txt'

data = np.loadtxt(FILENAME, delimiter=',')

# Plot for number of smoothing iterations vs path length
# plt.plot(data[4,0:], data[5,0:], label='Path Length')
# plt.xlabel('Smoothing Iterations')
# plt.ylabel('Path Length')
# plt.title('Smoothing Attemps vs Path Length')

# Plot for goal bias vs computation time
plt.plot(data[0,0:], data[1,0:], label='Time')
plt.xlabel('Goal Bias (%)')
plt.ylabel('Computation Time (s)')
plt.title('Computation Time vs Goal Bias')

plt.legend(loc='best')
axes = plt.gca()
plt.tight_layout()

# plt.savefig('smoothIterVsPathLen.png')
plt.savefig('compTimevsGoal.png')

# Show the plot
plt.show()
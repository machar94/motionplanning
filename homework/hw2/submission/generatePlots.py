# Import the necessary packages and modules
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({'font.size': 22})

# Prepare the data
t = np.linspace(0,50,1000)
y1 = np.full((1000), 1000000)
y2 = np.full((1000), 1000000)
astarSolutionTime = 24.85
astarSolutionCost = 11.39

anaStarSolutionTime1 = 9.04
anaStarSolutionTime2 = 18.70
anaStarSolutionCost1 = 22.23
anaStarSolutionCost2 = 15.70

for i in range(0,t.size):
    if t[i] > astarSolutionTime:
        y1[i] = astarSolutionCost
    if t[i] > anaStarSolutionTime1:
        y2[i] = anaStarSolutionCost1
    if t[i] > anaStarSolutionTime2:
        y2[i] = anaStarSolutionCost2

# Plot the data
plt.plot(t,y1, label='A*')
plt.plot(t,y2, label='ANA*')
plt.xlabel('Time')
plt.ylabel('Solution Cost')
plt.title('Path Planning Solution Time for Discrete Search Algorithms')

# Add a legend()
plt.legend()
axes = plt.gca()
axes.set_xlim([0,50])
axes.set_ylim([0,50])

# Show the plot
plt.show()

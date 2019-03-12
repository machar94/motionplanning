# Calculate mean and variance for the following:
# 1. Computation time for RRT
# 2. Computation time for smoothing
# 3. The number of nodes sampled
# 4. The length of the path unsmoothed
# 5. the lenght of the path smoothed

import numpy as np
import IPython
import os

# Numpy print options
np.set_printoptions(precision=2)
np.set_printoptions(suppress=True)

# Constants
FILENAME = 'goalBias.txt'

# Read in data
data = np.loadtxt(FILENAME, delimiter=',')

# Accidentally ran the code without resetting start time to current time
# This part might not be to be done all the time
compTime   = data[1,0:]
smoothTime = data[6,0:]

data[6,0:] = smoothTime - compTime

# Extract statistics
means     = np.average(data, axis=1)
variance  = np.var(data, axis=1)

print '==================================='
print 'RRT + Smoothing Stats:'
print '===================================\n'

print 'Computation Time RRT (Mean)      : %.2f' % means[1]
print 'Computation Time RRT (Var)       : %.2f' % variance[1]
print 'Computation Time Smoothing (Mean): %.2f' % means[6]
print 'Computation Time Smoothing (Var) : %.2f' % variance[6]
print 'Number of Samples (Mean)         : %.2f' % means[3]
print 'Number of Samples (Var)          : %.2f' % variance[3]
print 'Length of Unsmoothed Path (Mean) : %.2f' % means[7]
print 'Length of Unsmoothed Path (Var)  : %.2f' % variance[7]
print 'Length of Smoothed Path (Mean)   : %.2f' % means[5]
print 'Length of Smoothed Path (Var)    : %.2f' % variance[5]
print '\n==================================='


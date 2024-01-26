#!/usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
import os

data = np.genfromtxt("./step.csv", delimiter=" ", skip_header=0)

# times
t_real = data[:,0]
# Each swinging tip pos
p0 = data[:,1]
p1 = data[:,2]
p2 = data[:,3]

plt.figure()
plt.plot(t_real,p0,Label="Bez x")
plt.plot(t_real,p1,Label="Bez y")
plt.plot(t_real,p2,Label="Bez z")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("Bez")
plt.title("Bez - time")


# time = data[:,0]

# pCX = data[:,1]
# pCY = data[:,2]

# p0X = data[:,3]
# p0Y = data[:,4]

# p2X = data[:,5]
# p2Y = data[:,6]

# p1X = data[:,7]
# p1Y = data[:,8]

# p3X = data[:,9]
# p3Y = data[:,10]


# plt.figure()
# plt.plot(pCX,pCY,Label="Step C")

# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("Bez")
# plt.title("Bez - time")

# plt.figure()
# plt.plot(p0X,p0Y,Label="Step 0")

# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("Bez")
# plt.title("Bez - time")


# plt.figure()
# plt.plot(p2X,p2Y,Label="Step 2")

# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("Bez")
# plt.title("Bez - time")

# plt.figure()
# plt.plot(p1X,p1Y,Label="Step 1")

# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("Bez")
# plt.title("Bez - time")

# plt.figure()
# plt.plot(p3X,p3Y,Label="Step 3")

# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("Bez")
# plt.title("Bez - time")

plt.show()
plt.waitforbuttonpress(0) # this will wait for indefinite timeplt.close(fig)
plt.close('all')


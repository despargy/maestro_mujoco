#!/usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
import os

data = np.genfromtxt("./bez.csv", delimiter=" ", skip_header=0)

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

plt.figure()


plt.show()
plt.waitforbuttonpress(0) # this will wait for indefinite timeplt.close(fig)
plt.close('all')


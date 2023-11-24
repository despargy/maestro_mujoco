#!/usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
import os

os.chdir("../../data/")
data = np.genfromtxt("./data.csv", delimiter=" ", skip_header=1)

# times
t_real = data[:,0]

# CoM current pos
p0 = data[:,1]
p1 = data[:,2]
p2 = data[:,3]

# CoM desired pos
d_p0 = data[:,7]
d_p1 = data[:,8]
d_p2 = data[:,9]

plt.figure()
plt.plot(t_real,p0,Label="p x")
plt.plot(t_real,p1,Label="p y")
plt.plot(t_real,p2,Label="p z")

plt.plot(t_real,d_p0,Label="d_p x")
plt.plot(t_real,d_p1,Label="d_p y")
plt.plot(t_real,d_p2,Label="d_p z")


plt.legend()
plt.xlabel("t_real")
plt.ylabel("CoM position")
plt.title("CoM  - time")

# Error plots

ep0 = data[:,4]
ep1 = data[:,5]
ep2 = data[:,6]


plt.figure()
plt.plot(t_real,ep0,Label="p error x")
plt.plot(t_real,ep1,Label="p error y")
plt.plot(t_real,ep2,Label="p error z")
plt.legend()
plt.xlabel("t_real")
plt.ylabel("Error position")
plt.title("Error  - time")

eo0 = data[:,10]
eo1 = data[:,11]
eo2 = data[:,12]


plt.figure()
plt.plot(t_real,eo0,Label="ori error x")
plt.plot(t_real,eo1,Label="ori error y")
plt.plot(t_real,eo2,Label="ori error z")
plt.legend()
plt.xlabel("t_real")
plt.ylabel("Error ori")
plt.title("Error  - time")


plt.show()
plt.waitforbuttonpress(0) # this will wait for indefinite timeplt.close(fig)
plt.close('all')

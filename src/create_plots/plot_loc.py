#!/usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
import os

os.chdir("../../data/")
data = np.genfromtxt("./data.csv", delimiter=" ", skip_header=1)

# times
t_real = data[:,0]
# Each swinging tip pos
p0 = data[:,1]
p1 = data[:,2]
p2 = data[:,3]
# Weights for each tip (x-axis)
w0 = data[:,4]
w1 = data[:,5]
w2 = data[:,6]
w3 = data[:,7]
# Each swinging desired tip pos
p0d = data[:,8]
p1d = data[:,9]
p2d = data[:,10]

# CoM current pos
x1 = data[:,11]
y2 = data[:,12]
z3 = data[:,13]
# CoM targer pos
x1_d = data[:,14]
y2_d = data[:,15]
z3_d = data[:,16]

# Each swinging tip forces transformed
f0 = data[:,17]
f1 = data[:,18]
f2 = data[:,19]

plt.figure()
plt.plot(t_real,p0,Label="tip p x")
plt.plot(t_real,p1,Label="tip p y")
plt.plot(t_real,p2,Label="tip p z")

plt.plot(t_real,p0d,Label="tip pd x")
plt.plot(t_real,p1d,Label="tip pd y")
plt.plot(t_real,p2d,Label="tip pd z")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("Tip 0 position")
plt.title("Tip  - time")

plt.figure()


plt.plot(t_real,w0,Label="w0 x")
plt.plot(t_real,w1,Label="w1 x")
plt.plot(t_real,w2,Label="w2 x")
plt.plot(t_real,w3,Label="w3 x")


plt.legend()
plt.xlabel("t_real")
plt.ylabel("Weights position")
plt.title("Weights  - time")

plt.figure()


plt.plot(t_real,x1,Label="x actual")
plt.plot(t_real,y2,Label="y actual")
plt.plot(t_real,z3,Label="z actual")

plt.plot(t_real,x1_d,Label="x T")
plt.plot(t_real,y2_d,Label="y T")
plt.plot(t_real,z3_d,Label="z T")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("CoM pos")
plt.title("CoM  - time")

plt.figure()


plt.plot(t_real,w0/10000,Label="weight")
plt.plot(t_real,p0d,Label="p0d")
plt.plot(t_real,z3,Label="z actual")
plt.axvline(0.8, color='k',label='t0 swing')
plt.axvline(1.8, color='r',label='end swinging')
plt.axvline(1.1, color='b',label='t haalph swing swinging')

plt.legend()
plt.xlabel("t_real")
plt.ylabel("CoM pos")
plt.title("CoM  - time")


plt.figure()
plt.plot(t_real,f0,Label="f 0")
plt.plot(t_real,f1,Label="f 1")
plt.plot(t_real,f2,Label="f 2")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("Tip forces")
plt.title("Forces  - time")

plt.show()
plt.waitforbuttonpress(0) # this will wait for indefinite timeplt.close(fig)
plt.close('all')


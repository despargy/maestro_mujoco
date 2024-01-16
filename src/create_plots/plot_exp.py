#!/usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
import os

os.chdir("../../data/")
data = np.genfromtxt("./data.csv", delimiter=" ", skip_header=1)

# times
t_real = data[:,0]

# Weights for each tip (x-axis)
w0 = data[:,1]
w1 = data[:,2]
w2 = data[:,3]
w3 = data[:,4]

bx = data[:,5]
by = data[:,6]
bz = data[:,7]

tip_x = data[:,8]
tip_y = data[:,9]
tip_z = data[:,10]

px = data[:,11]
py = data[:,12]
pz = data[:,13]

pTx = data[:,14]
pTy = data[:,15]
pTz = data[:,16]

plt.plot(t_real,w0,Label="w0 x")
plt.plot(t_real,w1,Label="w1 x")
plt.plot(t_real,w2,Label="w2 x")
plt.plot(t_real,w3,Label="w3 x")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("Weights position")
plt.title("Weights  - time")

plt.figure()

plt.plot(t_real,bx,Label="bx world")
plt.plot(t_real,by,Label="by world")
plt.plot(t_real,bz,Label="bz world")
plt.plot(t_real,tip_x,Label="tipx world")
plt.plot(t_real,tip_y,Label="tipy world")
plt.plot(t_real,tip_z,Label="tipz world")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("Bezier world")
plt.title("Bezier  - time")

plt.figure()

plt.plot(t_real,px,Label="px world")
plt.plot(t_real,py,Label="py world")
plt.plot(t_real,pz,Label="pz world")
plt.plot(t_real,pTx,Label="pTx world")
plt.plot(t_real,pTy,Label="pTy world")
plt.plot(t_real,pTz,Label="pTz world")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("CoM world")
plt.title("CoM pos  - time")

plt.show()
plt.waitforbuttonpress(0) # this will wait for indefinite timeplt.close(fig)
plt.close('all')


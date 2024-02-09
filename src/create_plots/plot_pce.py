#!/usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
import os


data0 = np.genfromtxt("./pce_bag.csv", delimiter=" ", skip_header=0)
time = np.arange(0,len(data0[:,0]),1)
# times
ax_0 = data0[:,0]
ay_0 = data0[:,1]
az_0 = data0[:,2]
wx_0 = data0[:,3]
wy_0 = data0[:,4]
wz_0 = data0[:,5]

plt.figure()
plt.scatter(time, ax_0,Label="ax 0")
plt.legend()
plt.xlabel("samples")
plt.ylabel("ax0")
plt.title("")

plt.figure()
plt.scatter(time, ay_0,Label="ay 0")
plt.legend()
plt.xlabel("samples")
plt.ylabel("ay0")
plt.title("")

plt.figure()
plt.scatter(time, az_0,Label="az 0")
plt.legend()
plt.xlabel("samples")
plt.ylabel("az0")
plt.title("")

plt.figure()
plt.scatter(time, wx_0,Label="wx 0")
plt.legend()
plt.xlabel("samples")
plt.ylabel("wx0")
plt.title("")

plt.figure()
plt.scatter(time, wy_0,Label="wy 0")
plt.legend()
plt.xlabel("samples")
plt.ylabel("wy0")
plt.title("")

plt.figure()
plt.scatter(time, wz_0,Label="wz 0")
plt.legend()
plt.xlabel("samples")
plt.ylabel("wz0")
plt.title("")


plt.show()
plt.waitforbuttonpress(0) # this will wait for indefinite timeplt.close(fig)
plt.close('all')


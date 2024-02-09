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
pc_0 = data[:,1]
pc_1 = data[:,2]
pc_2 = data[:,3]
# Weights for each tip (x-axis)
w0 = data[:,4]
w1 = data[:,5]
w2 = data[:,6]
w3 = data[:,7]
# Each swinging desired tip pos
footgoal_0x = data[:,8]
footgoal_0y = data[:,9]
footgoal_0z = data[:,10]

footgoal_1x = data[:,11]
footgoal_1y = data[:,12]
footgoal_1z = data[:,13]

footgoal_2x = data[:,14]
footgoal_2y = data[:,15]
footgoal_2z = data[:,16]

footgoal_3x = data[:,17]
footgoal_3y = data[:,18]
footgoal_3z = data[:,19]

tip_0x = data[:,20]
tip_0y = data[:,21]
tip_0z = data[:,22]

tip_1x = data[:,23]
tip_1y = data[:,24]
tip_1z = data[:,25]

tip_2x = data[:,26]
tip_2y = data[:,27]
tip_2z = data[:,28]

tip_3x = data[:,29]
tip_3y = data[:,30]
tip_3z = data[:,31]

Bez_Ax = data[:,32]
Bez_Ay = data[:,33]
Bez_Az = data[:,34]

Bez_Bx = data[:,35]
Bez_By = data[:,36]
Bez_Bz = data[:,37]

# Each swinging tip pos
vel_x = data[:,38]
vel_y = data[:,39]
vel_z = data[:,40]


plt.figure()

plt.plot(t_real,w0,Label="w0")
plt.plot(t_real,w1,Label="w1")
plt.plot(t_real,w2,Label="w2")
plt.plot(t_real,w3,Label="w3")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("Weights")
plt.title("Weights  - time")

plt.figure()

plt.plot(t_real,pc_0,Label="pc x")
plt.plot(t_real,pc_1,Label="pc y")
plt.plot(t_real,pc_2,Label="pc z")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("CoM Pos")
plt.title("pos  - time")


plt.figure()

# plt.plot(t_real,w0/10000,Label="weight 0")
# plt.plot(t_real,w3/10000,Label="weight 3")

plt.plot(t_real,10*tip_0z,Label="swing tip 0")
plt.plot(t_real,10*tip_3z,Label="swing tip 3")

# plt.plot(t_real,pc_2,Label="z actual")
plt.axvline(0.03, color='k',label='t0 swing')
plt.axvline(10.3, color='b',label='t half swing swinging')
plt.axvline(10.8, color='r',label='end swinging')

plt.legend()
plt.xlabel("t_real")
plt.ylabel("CoM pos")
plt.title("CoM  - time")



plt.figure()
# plt.plot(t_real,footgoal_0z,Label="Foot goal 0 z ")
# plt.plot(t_real,footgoal_1z,Label="Foot goal 1 z ")
# plt.plot(t_real,footgoal_2z,Label="Foot goal 2 z ")
# plt.plot(t_real,footgoal_3z,Label="Foot goal 3 z ")

plt.plot(t_real, tip_0z, label="0 z")
plt.plot(t_real, tip_1z, label="1 z")
plt.plot(t_real, tip_2z, label="2 z")
plt.plot(t_real, tip_3z, label="3 z")

plt.plot(t_real, Bez_Az, label="Bez A z")
plt.plot(t_real, Bez_Bz, label="Bez B z")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("Tip z")
plt.title("tip z  - time")


plt.figure()
plt.plot(t_real,footgoal_0x,Label="Foot goal 0 x ")
plt.plot(t_real,footgoal_1x,Label="Foot goal 1 x ")
plt.plot(t_real,footgoal_2x,Label="Foot goal 2 x ")
plt.plot(t_real,footgoal_3x,Label="Foot goal 3 x ")

plt.plot(t_real, tip_0x, label="0 x")
plt.plot(t_real, tip_1x, label="1 x")
plt.plot(t_real, tip_2x, label="2 x")
plt.plot(t_real, tip_3x, label="3 x")

plt.plot(t_real, Bez_Ax, label="Bez A x")
plt.plot(t_real, Bez_Bx, label="Bez B x")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("Tip x")
plt.title("tip x  - time")



plt.figure()
plt.plot(t_real,footgoal_0y,Label="Foot goal 0 y ")
plt.plot(t_real,footgoal_1y,Label="Foot goal 1 y ")
plt.plot(t_real,footgoal_2y,Label="Foot goal 2 y ")
plt.plot(t_real,footgoal_3y,Label="Foot goal 3 y ")

plt.plot(t_real, tip_0y, label="0 y")
plt.plot(t_real, tip_1y, label="1 y")
plt.plot(t_real, tip_2y, label="2 y")
plt.plot(t_real, tip_3y, label="3 y")

plt.plot(t_real, Bez_Ay, label="Bez A y")
plt.plot(t_real, Bez_By, label="Bez B y")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("Tip y")
plt.title("tip y  - time")

plt.figure()

plt.plot(t_real,vel_x,Label="vel x")
plt.plot(t_real,vel_y,Label="vel y")
plt.plot(t_real,vel_z,Label="vel z")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("CoM Vel")
plt.title("vel  - time")


# 3D Plots for Com tips 


# plt.figure()
# ax = plt.axes(projection='3d')
# ax.scatter3D(tip_0x, tip_0y, tip_0z, c=tip_0z, cmap='Greens');
# ax.scatter3D(tip_1x, tip_1y, tip_1z, c=tip_1z, cmap='Blues');
# ax.scatter3D(tip_2x, tip_2y, tip_2z, c=tip_2z, cmap='Reds');
# ax.scatter3D(tip_3x, tip_3y, tip_3z, c=tip_3z, cmap='Purples');
# ax.scatter3D(pc_0, pc_1, pc_2, c=pc_2, cmap='Greys');


plt.show()
plt.waitforbuttonpress(0) # this will wait for indefinite timeplt.close(fig)
plt.close('all')


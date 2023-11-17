#!/usr/bin/env python3
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import math 
from mpl_toolkits import mplot3d

data = np.genfromtxt("/home/despinar/mujoco_ws/maestro_mujoco/data/data.csv", delimiter=" ", skip_header=1) #TODO

# times
t_real = data[:,0]

p0 = data[:,1]
p1 = data[:,2]
p2 = data[:,3]

w0 = data[:,4]
w1 = data[:,5]
w2 = data[:,6]
w3 = data[:,7]

p0d = data[:,8]
p1d = data[:,9]
p2d = data[:,10]
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

# fig = plt.figure()
# ax = plt.axes(projection='3d')
# ax.scatter3D(p0, p1, p2, c=p2, cmap='Greens')



# # Error plots

# ep0 = data[:,4]
# ep1 = data[:,5]
# ep2 = data[:,6]


# plt.figure()
# plt.plot(t_real,ep0,Label="p error x")
# plt.plot(t_real,ep1,Label="p error y")
# plt.plot(t_real,ep2,Label="p error z")
# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("Error position")
# plt.title("Error  - time")


# f0 = data[:,1]
# f1 = data[:,4]
# f2 = data[:,7]
# f3 = data[:,10]


# plt.figure()
# plt.plot(t_real,f0)
# plt.plot(t_real,f1)
# plt.plot(t_real,f2)
# plt.plot(t_real,f3)
# plt.xlabel("t_real")
# plt.ylabel("Foot forces")
# plt.title("Forces  - time")



# tip0_X = data[:,1]
# tip1_X = data[:,4]
# tip2_X = data[:,7]
# tip3_X = data[:,10]


# plt.figure()
# plt.plot(t_real,tip0_X)
# plt.plot(t_real,tip1_X)
# plt.plot(t_real,tip2_X)
# plt.plot(t_real,tip3_X)
# plt.xlabel("t_real")
# plt.ylabel("Tip pose X")
# plt.title("Tip  - time")


# tip0_y = data[:,2]
# tip1_y = data[:,5]
# tip2_y = data[:,8]
# tip3_y = data[:,11]


# plt.figure()
# plt.plot(t_real,tip0_y)
# plt.plot(t_real,tip1_y)
# plt.plot(t_real,tip2_y)
# plt.plot(t_real,tip3_y)
# plt.xlabel("t_real")
# plt.ylabel("Tip pose Y")
# plt.title("Tip  - time")

# tip0_z = data[:,3]
# tip1_z = data[:,6]
# tip2_z = data[:,9]
# tip3_z = data[:,12]


# plt.figure()
# plt.plot(t_real,tip0_z)
# plt.plot(t_real,tip1_z)
# plt.plot(t_real,tip2_z)
# plt.plot(t_real,tip3_z)
# plt.xlabel("t_real")
# plt.ylabel("Tip pose Z")
# plt.title("Tip  - time")

plt.show()
plt.waitforbuttonpress(0) # this will wait for indefinite timeplt.close(fig)
plt.close('all')


# FOOT FORCES NORM  
# # f norm 0
# f0_all = data[:,1:4]
# print(f0_all)
# f0_norm = np.empty(shape=np.shape(t_real))
# for i in range(np.shape(t_real)[0]):
#     f0_norm[i] = math.sqrt( f0_all[i,0]**2 + f0_all[i,1]**2 + f0_all[i,2]**2 )

# # f norm 1
# f1_all = data[:,4:7]
# f1_norm = np.empty(shape=np.shape(t_real))
# for i in range(np.shape(t_real)[0]):
#     f1_norm[i] = math.sqrt( f1_all[i,0]**2 + f1_all[i,1]**2 + f1_all[i,2]**2 )

# # f norm 2
# f2_all = data[:,7:10]
# f2_norm = np.empty(shape=np.shape(t_real))
# for i in range(np.shape(t_real)[0]):
#     f2_norm[i] = math.sqrt( f2_all[i,0]**2 + f2_all[i,1]**2 + f2_all[i,2]**2 )

# # f norm 3
# f3_all = data[:,10:13]
# f3_norm = np.empty(shape=np.shape(t_real))
# for i in range(np.shape(t_real)[0]):
#     f3_norm[i] = math.sqrt( f3_all[i,0]**2 + f3_all[i,1]**2 + f3_all[i,2]**2 )

# plt.figure()
# plt.plot(t_real,f0_norm)
# plt.plot(t_real,f1_norm)
# plt.plot(t_real,f2_norm)
# plt.plot(t_real,f3_norm)
# plt.xlabel("t_real")
# plt.ylabel("Foot forces NORM")
# plt.title("Forces NORM - time")


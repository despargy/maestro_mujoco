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

# Each tip prob stability
prob_0 = data[:,8]
prob_1 = data[:,9]
prob_2 = data[:,10]
prob_3 = data[:,11]

# Stance weights and prob
w_stance_A = data[:,12]
w_stance_B = data[:,13]
prob_stance_A = data[:,14]
prob_stance_B = data[:,15]

# Swing weights and prob
w_swing_A = data[:,16]
w_swing_B = data[:,17]
prob_swing_A = data[:,18]
prob_swing_B = data[:,19]

# Each swinging tip pos
vel_x = data[:,20]
vel_y = data[:,21]
vel_z = data[:,22]

imu_x = data[:,20]
imu_y = data[:,21]
imu_z = data[:,22]


# plt.figure()

# plt.plot(t_real,pc_0,Label="pc x")
# plt.plot(t_real,pc_1,Label="pc y")
# plt.plot(t_real,pc_2,Label="pc z")

# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("CoM Pos")
# plt.title("pos  - time")

# plt.figure()

# plt.plot(t_real,vel_x,Label="vel_x")
# plt.plot(t_real,vel_y,Label="vel_y")
# plt.plot(t_real,vel_z,Label="vel_z")

# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("CoM Vel")
# plt.title("vel  - time")

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

plt.plot(t_real,prob_0,Label="prob 0")
plt.plot(t_real,prob_1,Label="prob 1")
plt.plot(t_real,prob_2,Label="prob 2")
plt.plot(t_real,prob_3,Label="prob 3")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("Prob")
plt.title("Prob  - time")


plt.figure()

plt.plot(t_real,w_stance_A/2000,Label="Stance A weight scaled")
plt.plot(t_real,w_stance_B/2000,Label="Stance B weight scaled")
plt.plot(t_real,prob_stance_A,Label="Stance A prob")
plt.plot(t_real,prob_stance_B,Label="Stance B prob")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("Stance")
plt.title("Stance  - time")

# plt.figure()

# plt.plot(t_real,w_swing_A/70000,Label="Swing A weight scaled")
# plt.plot(t_real,w_swing_B/70000,Label="Swing B weight scaled")
# plt.plot(t_real,prob_swing_A,Label="Swing A prob")
# plt.plot(t_real,prob_swing_B,Label="Swing B prob")

# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("Swing")
# plt.title("Swing  - time")


# plt.figure()

# plt.plot(t_real,imu_x,Label="acc x")
# plt.plot(t_real,imu_y,Label="acc y")
# plt.plot(t_real,imu_z,Label="acc z")

# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("CoM Acc")
# plt.title("acc  - time")


plt.show()
plt.waitforbuttonpress(0) # this will wait for indefinite timeplt.close(fig)
plt.close('all')


#!/usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
import os

os.chdir("../../data/")
data = np.genfromtxt("./data_adapt_1.csv", delimiter=" ", skip_header=1)

data_slip = np.genfromtxt("./data_slip_1.csv", delimiter=" ", skip_header=1)

num = np.min( [(len(data)), (len(data_slip))])

data = data[0:num,:]
data_slip = data_slip[0:num,:]

# times
t_real = data[:,0]
########################################################################
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
er_vel_x = data[:,20]
er_vel_y = data[:,21]
er_vel_z = data[:,22]

imu_x = data[:,20]
imu_y = data[:,21]
imu_z = data[:,22]

fA_x = data[:,23]
fA_y = data[:,24]
fA_z = data[:,25]
fB_x = data[:,26]
fB_y = data[:,27]
fB_z = data[:,28]

vx = data[:,29] 
vy = data[:,30]
vz = data[:,31]

##################################################################



# Each swinging tip pos
pc_0_slip = data_slip[:,1]
pc_1_slip = data_slip[:,2]
pc_2_slip = data_slip[:,3]

# Weights for each tip (x-axis)
w0_slip = data_slip[:,4]
w1_slip = data_slip[:,5]
w2_slip = data_slip[:,6]
w3_slip = data_slip[:,7]

# Each tip prob stability
prob_0_slip = data_slip[:,8]
prob_1_slip = data_slip[:,9]
prob_2_slip = data_slip[:,10]
prob_3_slip = data_slip[:,11]

# Stance weights and prob
w_stance_A_slip = data_slip[:,12]
w_stance_B_slip = data_slip[:,13]
prob_stance_A_slip = data_slip[:,14]
prob_stance_B_slip = data_slip[:,15]

# Swing weights and prob
w_swing_A_slip = data_slip[:,16]
w_swing_B_slip = data_slip[:,17]
prob_swing_A_slip = data_slip[:,18]
prob_swing_B_slip = data_slip[:,19]

# Each swinging tip pos
er_vel_x_slip = data_slip[:,20]
er_vel_y_slip = data_slip[:,21]
er_vel_z_slip = data_slip[:,22]

imu_x_slip = data_slip[:,20]
imu_y_slip = data_slip[:,21]
imu_z_slip = data_slip[:,22]

fA_x_slip = data_slip[:,23]
fA_y_slip = data_slip[:,24]
fA_z_slip = data_slip[:,25]
fB_x_slip = data_slip[:,26]
fB_y_slip = data_slip[:,27]
fB_z_slip = data_slip[:,28]

vx_slip = data_slip[:,29] 
vy_slip = data_slip[:,30]
vz_slip = data_slip[:,31]

##################################################################
plt.figure()

plt.plot(t_real,pc_0,Label="pc x")
# plt.plot(t_real,pc_1,Label="pc y")
# plt.plot(t_real,pc_2,Label="pc z")
plt.plot(t_real,pc_0_slip,Label="slip pc x")
# plt.plot(t_real,pc_1_slip,Label="slip pc y")
# plt.plot(t_real,pc_2_slip,Label="slip pc z")
plt.legend()
plt.xlabel("t_real")
plt.ylabel("CoM Pos")
plt.title("pos X - time")

plt.figure()

# plt.plot(t_real,pc_0,Label="pc x")
plt.plot(t_real,pc_1,Label="pc y")
# plt.plot(t_real,pc_2,Label="pc z")
# plt.plot(t_real,pc_0_slip,Label="slip pc x")
plt.plot(t_real,pc_1_slip,Label="slip pc y")
# plt.plot(t_real,pc_2_slip,Label="slip pc z")
plt.legend()
plt.xlabel("t_real")
plt.ylabel("CoM Pos")
plt.title("pos Y - time")

plt.figure()

# plt.plot(t_real,pc_0,Label="pc x")
# plt.plot(t_real,pc_1,Label="pc y")
plt.plot(t_real,pc_2,Label="pc z")
# plt.plot(t_real,pc_0_slip,Label="slip pc x")
# plt.plot(t_real,pc_1_slip,Label="slip pc y")
plt.plot(t_real,pc_2_slip,Label="slip pc z")
plt.legend()
plt.xlabel("t_real")
plt.ylabel("CoM Pos")
plt.title("pos Z - time")
# plt.figure()

# plt.plot(t_real,vel_x,Label="vel_x")
# plt.plot(t_real,vel_x_slip,Label="slip vel_x")

# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("CoM error Vel")
# plt.title("vel x  - time")

# plt.figure()

# plt.plot(t_real,er_vel_y,Label="vel_y")
# plt.plot(t_real,er_vel_y_slip,Label="slip vel_y")
# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("CoM error Vel")
# plt.title("vel y - time")

# plt.figure()


# plt.plot(t_real,er_vel_z,Label="vel_z")

# plt.plot(t_real,er_vel_z_slip,Label="slip vel_z")
# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("CoM error Vel")
# plt.title("vel z  - time")

# plt.figure()

# plt.plot(t_real,w0,Label="w0")
# plt.plot(t_real,w1,Label="w1")
# plt.plot(t_real,w2,Label="w2")
# plt.plot(t_real,w3,Label="w3")
# plt.plot(t_real,w0_slip,Label="slip w0")
# plt.plot(t_real,w1_slip,Label="slip w1")
# plt.plot(t_real,w2_slip,Label="slip w2")
# plt.plot(t_real,w3_slip,Label="slip w3")
# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("Weights")
# plt.title("Weights  - time")



plt.figure()

plt.plot(t_real,w0/10000000,Label="w0 scaled")
plt.plot(t_real,w0_slip/10000000,Label="slip w0 scaled")
plt.plot(t_real,prob_0,Label="prob 0")
plt.plot(t_real,prob_0_slip,Label="slip prob 0")
plt.legend()
plt.xlabel("t_real")
plt.ylabel("Weights")
plt.title("w0 - time")


plt.figure()

plt.plot(t_real,w1/10000000,Label="w1 scaled")
plt.plot(t_real,w1_slip/10000000,Label="slip w1 scaled")

plt.plot(t_real,prob_1,Label="prob 1")
plt.plot(t_real,prob_1_slip,Label="slip prob 1")
plt.legend()
plt.xlabel("t_real")
plt.ylabel("Weights")
plt.title("w1  - time")


plt.figure()


plt.plot(t_real,w2/10000000,Label="w2 scaled")
plt.plot(t_real,w2_slip/10000000,Label="slip w2 scaled")
plt.plot(t_real,prob_2,Label="prob 2")
plt.plot(t_real,prob_2_slip,Label="slip prob 2")
plt.legend()
plt.xlabel("t_real")
plt.ylabel("Weights")
plt.title("w2  - time")


plt.figure()

plt.plot(t_real,w3/10000000,Label="w3 scaled")
plt.plot(t_real,w3_slip/10000000,Label="slip w3 scaled")
plt.plot(t_real,prob_3,Label="prob 3")
plt.plot(t_real,prob_3_slip,Label="slip prob 3")
plt.legend()
plt.xlabel("t_real")
plt.ylabel("Weights")
plt.title("w3  - time")

# plt.figure()

# plt.plot(t_real,prob_0,Label="prob 0")
# plt.plot(t_real,prob_0_slip,Label="slip prob 0")

# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("Prob")
# plt.title("Prob 0 - time")

# plt.figure()

# plt.plot(t_real,prob_1,Label="prob 1")
# plt.plot(t_real,prob_1_slip,Label="slip prob 1")

# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("Prob")
# plt.title("Prob 1 - time")

# plt.figure()
# plt.plot(t_real,prob_2,Label="prob 2")
# plt.plot(t_real,prob_2_slip,Label="slip prob 2")
# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("Prob")
# plt.title("Prob 2 - time")

# plt.figure()

# plt.plot(t_real,prob_3,Label="prob 3")
# plt.plot(t_real,prob_3_slip,Label="slip prob 3")

# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("Prob")
# plt.title("Prob 3  - time")



# plt.figure()

# plt.plot(t_real,w_stance_A/50,Label="Stance A weight scaled")
# plt.plot(t_real,w_stance_B/50,Label="Stance B weight scaled")
# plt.plot(t_real,prob_stance_A,Label="Stance A prob")
# plt.plot(t_real,prob_stance_B,Label="Stance B prob")
# plt.plot(t_real,w_stance_A_slip/50,Label="slip Stance A weight scaled")
# plt.plot(t_real,w_stance_B_slip/50,Label="slip Stance B weight scaled")
# plt.plot(t_real,prob_stance_A_slip,Label="slip Stance A prob")
# plt.plot(t_real,prob_stance_B_slip,Label="slip Stance B prob")
# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("Stance")
# plt.title("Stance  - time")

plt.figure()

plt.plot(t_real,w_swing_A/10000000,Label="Swing A weight scaled")
# plt.plot(t_real,w_swing_B/10000000,Label="Swing B weight scaled")
plt.plot(t_real,prob_swing_A,Label="Swing A prob")
# plt.plot(t_real,prob_swing_B,Label="Swing B prob")

plt.plot(t_real,w_swing_A_slip/10000000,Label="slip Swing A weight scaled")
# plt.plot(t_real,w_swing_B_slip/10000000,Label="slip Swing B weight scaled")
plt.plot(t_real,prob_swing_A_slip,Label="slip Swing A prob")
# plt.plot(t_real,prob_swing_B_slip,Label="slip Swing B prob")
plt.legend()
plt.xlabel("t_real")
plt.ylabel("Swing")
plt.title("Swing A - time")


plt.figure()

# plt.plot(t_real,w_swing_A/10000000,Label="Swing A weight scaled")
plt.plot(t_real,w_swing_B/10000000,Label="Swing B weight scaled")
# plt.plot(t_real,prob_swing_A,Label="Swing A prob")
plt.plot(t_real,prob_swing_B,Label="Swing B prob")

# plt.plot(t_real,w_swing_A_slip/10000000,Label="slip Swing A weight scaled")
plt.plot(t_real,w_swing_B_slip/10000000,Label="slip Swing B weight scaled")
# plt.plot(t_real,prob_swing_A_slip,Label="slip Swing A prob")
plt.plot(t_real,prob_swing_B_slip,Label="slip Swing B prob")
plt.legend()
plt.xlabel("t_real")
plt.ylabel("Swing")
plt.title("Swing B - time")

plt.figure()

# plt.plot(t_real,fA_x,Label="fA_x ")
# plt.plot(t_real,fA_y,Label="fA_y ")
plt.plot(t_real,fA_z,Label="fA_z ")

# plt.plot(t_real,fB_x,Label="fB_x ")
# plt.plot(t_real,fB_y,Label="fB_y ")
plt.plot(t_real,fB_z,Label="fB_z ")
# 
# plt.plot(t_real,fA_x_slip,Label="slip fA_x ")
# plt.plot(t_real,fA_y_slip,Label="slip fA_y ")
plt.plot(t_real,fA_z_slip,Label="slip fA_z ")
# plt.plot(t_real,fB_x_slip,Label="slip fB_x ")
# plt.plot(t_real,fB_y_slip,Label="slip fB_y ")
plt.plot(t_real,fB_z_slip,Label="slip fB_z ")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("Forces")
plt.title("Forces swingingqqqq  - time")


plt.figure()

plt.plot(t_real,vx,Label="vx")
plt.plot(t_real,vy,Label="vy")
plt.plot(t_real,vz,Label="vz")

plt.plot(t_real,vx_slip,Label="vx_slip")
plt.plot(t_real,vy_slip,Label="vy_slip")
plt.plot(t_real,vz_slip,Label="vz_slip")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("CoM Vel")
plt.title("Vel  - time")

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


#!/usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
import os
import csv

SAVE_CSV = False

os.chdir("../../data/State-PP")
num = 19310
data_Fc = np.genfromtxt("./data-Fc.csv", delimiter=" ", skip_header=1)
data_dq_out = np.genfromtxt("./data-dq_out.csv", delimiter=" ", skip_header=1)
data_q_out = np.genfromtxt("./data-q_out.csv", delimiter=" ", skip_header=1)
data_joint_pos = np.genfromtxt("./data-joint-pos.csv", delimiter=" ", skip_header=1)
data_joint_vel = np.genfromtxt("./data-joint-vel.csv", delimiter=" ", skip_header=1)
data_tau = np.genfromtxt("./data-tau.csv", delimiter=" ", skip_header=1)
data_PP = np.genfromtxt("./data-PP.csv", delimiter=" ", skip_header=1)
data_CoM = np.genfromtxt("./data-CoM.csv", delimiter=" ", skip_header=1)

data_Fc = data_Fc[:num]
data_dq_out = data_dq_out[:num]
data_q_out = data_q_out[:num]
data_joint_pos = data_joint_pos[:num]
data_joint_vel = data_joint_vel[:num]
data_tau = data_tau[:num]
data_PP = data_PP[:num]
data_CoM = data_CoM[:num]



t_real = data_Fc[:,0]
phase_id = data_Fc[:,1]
# Fc
Fc = np.zeros((6,data_Fc[:,2].shape[0]))
for i,f in enumerate(Fc):
    Fc[i,:] = data_Fc[:,i+2]
print(Fc.shape)
# dq out
dq_out = np.zeros((12,data_dq_out[:,2].shape[0]))
for i,f in enumerate(dq_out):
    dq_out[i,:] = data_dq_out[:,i+2]
# q out
q_out = np.zeros((12,data_q_out[:,2].shape[0]))
for i,f in enumerate(q_out):
    q_out[i,:] = data_q_out[:,i+2]
# joint pos
joint_pos = np.zeros((12,data_joint_pos[:,2].shape[0]))
for i,f in enumerate(joint_pos):
    joint_pos[i,:] = data_joint_pos[:,i+2]
# joint vel
joint_vel = np.zeros((12,data_joint_vel[:,2].shape[0]))
for i,f in enumerate(joint_vel):
    joint_vel[i,:] = data_joint_vel[:,i+2]
# tau
tau = np.zeros((12,data_tau[:,2].shape[0]))
for i,f in enumerate(tau):
    tau[i,:] = data_tau[:,i+2]
# PP
prob_stable = np.zeros((4,data_PP[:,2].shape[0]))
for i,f in enumerate(prob_stable):
    prob_stable[i,:] = data_PP[:,i+2]
vv = np.zeros((12,data_PP[:,6].shape[0]))
for i,f in enumerate(vv):
    vv[i,:] = data_PP[:,i+6]

# CoM
pc = np.zeros((3,data_CoM[:,2].shape[0]))
for i,f in enumerate(pc):
    pc[i,:] = data_CoM[:,i+2]
dpc = np.zeros((3,data_CoM[:,5].shape[0]))
for i,f in enumerate(pc):
    dpc[i,:] = data_CoM[:,i+5]
print(data_CoM.shape)

if SAVE_CSV:
    merged_data = np.column_stack((t_real,phase_id))
    for f in Fc:
        merged_data = np.column_stack((merged_data,f))
    for f in dq_out:
        merged_data = np.column_stack((merged_data,f))
    for f in q_out:
        merged_data = np.column_stack((merged_data,f))
    for f in joint_pos:
        merged_data = np.column_stack((merged_data,f))
    for f in joint_vel:
        merged_data = np.column_stack((merged_data,f))
    for f in tau:
        merged_data = np.column_stack((merged_data,f))
    for f in prob_stable:
        merged_data = np.column_stack((merged_data,f)) 
    for f in vv:
        merged_data = np.column_stack((merged_data,f)) 
    for f in pc:
        merged_data = np.column_stack((merged_data,f)) 
    for f in dpc:
        merged_data = np.column_stack((merged_data,f))  

    row_names = ["t_real", "phase_id"]
    for i in range(6):
        row_names.append("Fc_%i"%i)
    for i in range(12):
        row_names.append("dq_out_%i"%i)
    for i in range(12):
        row_names.append("q_out_%i"%i)
    for i in range(12):
        row_names.append("joint_pos_%i"%i)
    for i in range(12):
        row_names.append("joint_vel_%i"%i)
    for i in range(12):
        row_names.append("tau_%i"%i)
    for i in range(4):
        row_names.append("prob_stable_%i"%i)
    for i in range(12):
        row_names.append("vv_%i"%i)
    for i in range(3):
        row_names.append("pc_%i"%i)
    for i in range(3):
        row_names.append("dpc_%i"%i)

    # Save arrays as columns in CSV
    with open("total.csv", "w", newline="") as f:
        writer = csv.writer(f, delimiter='\t')
        writer.writerow(row_names)
        writer.writerows(merged_data)  # Write rows of paired values

# plt.figure()
# plt.plot(t_real, Fc[0,:], label="Fc 0")
# plt.plot(t_real, Fc[1,:], label="Fc 0")
# plt.plot(t_real, Fc[2,:], label="Fc 0")
# plt.plot(t_real, Fc[3,:], label="Fc 0")
# plt.plot(t_real, Fc[4,:], label="Fc 0")
# plt.plot(t_real, Fc[5,:], label="Fc 0")
# plt.fill_between(t_real, -100,200, where= phase_id == 1, facecolor='green', alpha=.2)
# plt.fill_between(t_real, -100,200, where= phase_id == 0, facecolor='red', alpha=.2)
# plt.title("Fc")

# for i in range(12):
#     plt.figure()
#     plt.plot(t_real, dq_out[i,:], label="dq_out")
#     plt.fill_between(t_real, -20,20, where= phase_id == 1, facecolor='green', alpha=.2)
#     plt.fill_between(t_real, -20,20, where= phase_id == 0, facecolor='red', alpha=.2)
#     plt.title("dq_out")


# for i in range(12):
#     plt.figure()
#     plt.plot(t_real, q_out[i,:], label="q_out")
#     plt.fill_between(t_real, -5,5, where= phase_id == 1, facecolor='green', alpha=.2)
#     plt.fill_between(t_real, -5,5, where= phase_id == 0, facecolor='red', alpha=.2)
#     plt.title("q_out")

# for i in range(12):
#     plt.figure()
#     plt.plot(t_real, joint_pos[i,:], label="joint_pos")
#     plt.fill_between(t_real, -5,5, where= phase_id == 1, facecolor='green', alpha=.2)
#     plt.fill_between(t_real, -5,5, where= phase_id == 0, facecolor='red', alpha=.2)
#     plt.title("joint_pos")


# for i in range(12):
#     plt.figure()
#     plt.plot(t_real, joint_vel[i,:], label="joint_vel")
#     plt.fill_between(t_real, -5,5, where= phase_id == 1, facecolor='green', alpha=.2)
#     plt.fill_between(t_real, -5,5, where= phase_id == 0, facecolor='red', alpha=.2)
#     plt.title("joint_vel")

# for i in range(12):
#     plt.figure()
#     plt.plot(t_real, tau[i,:], label="tau")
#     plt.fill_between(t_real, -5,5, where= phase_id == 1, facecolor='green', alpha=.2)
#     plt.fill_between(t_real, -5,5, where= phase_id == 0, facecolor='red', alpha=.2)
#     plt.title("tau")   

# for i in range(4):
#     plt.figure()
#     plt.plot(t_real, prob_stable[i,:], label="prob_stable")
#     plt.fill_between(t_real, 1, where= phase_id == 1, facecolor='green', alpha=.2)
#     plt.fill_between(t_real, 1, where= phase_id == 0, facecolor='red', alpha=.2)
#     plt.title("prob_stable")   

# for i in range(12):
#     plt.figure()
#     plt.plot(t_real, vv[i,:], label="vv")
#     plt.fill_between(t_real, 1000, where= phase_id == 1, facecolor='green', alpha=.2)
#     plt.fill_between(t_real, 1000, where= phase_id == 0, facecolor='red', alpha=.2)
#     plt.ylim(50,1000)
#     plt.title("vv")   

# for i in range(3):
#     plt.figure()
#     plt.plot(t_real, pc[i,:], label="pc")
#     plt.fill_between(t_real, 1, where= phase_id == 1, facecolor='green', alpha=.2)
#     plt.fill_between(t_real, 1, where= phase_id == 0, facecolor='red', alpha=.2)
#     plt.title("pc")   

# for i in range(3):
#     plt.figure()
#     plt.plot(t_real, dpc[i,:], label="dpc")
#     plt.fill_between(t_real, 1, where= phase_id == 1, facecolor='green', alpha=.2)
#     plt.fill_between(t_real, 1, where= phase_id == 0, facecolor='red', alpha=.2)
#     plt.title("dpc")  
###############################################################################




plt.show()
plt.waitforbuttonpress(0) # this will wait for indefinite timeplt.close(fig)
plt.close('all')


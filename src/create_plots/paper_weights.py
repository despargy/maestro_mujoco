#!/usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
import os


os.chdir("../../data/")
# data = np.genfromtxt("./data.csv", delimiter=" ", skip_header=1)

# data_mpc = np.genfromtxt("./data_mpc_1.csv", delimiter=" ", skip_header=1)
# Initialize an empty list to hold the valid rows
data_mpc = []

# Open the file and read it line by line
with open("./data_mpc_1.csv", 'r') as file:
    for line in file:
        columns = line.strip().split(" ")
        if len(columns) == 7:
            try:
                # Try converting all columns to float
                float_columns = [float(value) for value in columns]
                data_mpc.append(float_columns)
            except ValueError:
                continue
            
# Convert the list to a NumPy array
data_mpc = np.array(data_mpc)

data = np.genfromtxt("./data_adapt_3.csv", delimiter=" ", skip_header=1)
data_slip = np.genfromtxt("./data_slip_3.csv", delimiter=" ", skip_header=1)
num = np.min( [(len(data)), (len(data_slip))])
# num2 = np.min( [(len(data)), (len(data_mpc))])
# num = min(num1, num2)
data = data[0:num,:]
data_slip = data_slip[0:num,:]

data_mpc = data_mpc[0:num]
# MPC
# times
t_real_mpc = data_mpc[:,0]

# CoM current pos
pc_0_mpc = data_mpc[:,1]
pc_1_mpc = data_mpc[:,2]
pc_2_mpc = data_mpc[:,3]

# CoM desired pos
vx_mpc = data_mpc[:,4]
vy_mpc = data_mpc[:,5]
vz_mpc = data_mpc[:,6]



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
er_vel_x = data[:,20]
er_vel_y = data[:,21]
er_vel_z = data[:,22]

fA_z = data[:,23]
fB_z = data[:,24]

fA_stance_z = data[:,25]
fB_stance_z = data[:,26]

vx = data[:,27] 
vy = data[:,28]
vz = data[:,29]


# times
t_real_slip = data_slip[:,0]
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

fA_z_slip = data_slip[:,23]
fB_z_slip = data_slip[:,24]

fA_stance_z_slip = data_slip[:,25]
fB_stance_z_slip = data_slip[:,26]

vx_slip = data_slip[:,27] 
vy_slip = data_slip[:,28]
vz_slip = data_slip[:,29]

plt.figure()

plt.plot(t_real_mpc,pc_0_mpc,Label="MPC pc x")
plt.plot(t_real_mpc,pc_1_mpc,Label="MPC pc y")
plt.plot(t_real_mpc,pc_2_mpc,Label="MPC pc z")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("CoM Pos")
plt.title("pos  - time")


plt.figure()

plt.plot(t_real_mpc,vx_mpc,Label="MPC vx")
plt.plot(t_real_mpc,vy_mpc,Label="MPC vy")
plt.plot(t_real_mpc,vz_mpc,Label="MPC vz")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("MPC CoM Vel")
plt.title("Vel MPC  - time")





plt.figure()

plt.plot(t_real[3025:3200],w0[3025:3200],Label="$w_0$")
plt.plot(t_real[3025:3200],w1[3025:3200],Label="$w_1$")
plt.plot(t_real[3025:3200],w2[3025:3200],Label="$w_2$")
plt.plot(t_real[3025:3200],w3[3025:3200],Label="$w_3$")

plt.axvline(x=6.202, color='k', linestyle='dashed', Label="$t_D$")
plt.axvline(x=6.374, color='k', linestyle='dashed')
plt.axvline(x=6.378, color='k', linestyle='dashed')

plt.fill_between(t_real[3025:3200], 10000000, where=w3[3025:3200] > 90, facecolor='red', alpha=.2)
plt.fill_between(t_real[3025:3200], 10000000, where=w1[3025:3200] > 90, facecolor='green', alpha=.2)

plt.legend(loc='upper left')
plt.xlabel("Time (s)")
plt.ylabel("Weights $x-axis$")
plt.title("Swinging weights")


# plt.figure()

# plt.plot(t_real,prob_0,Label="prob 0")
# plt.plot(t_real,prob_1,Label="prob 1")
# plt.plot(t_real,prob_2,Label="prob 2")
# plt.plot(t_real,prob_3,Label="prob 3")

# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("Prob")
# plt.title("Prob  - time")


# plt.figure()

# plt.plot(t_real,w_stance_A/max(w_stance_A),Label="Stance A weight scaled")
# plt.plot(t_real,prob_stance_A,Label="Stance A prob")

# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("Stance")
# plt.title("Stance A - time")

# plt.figure()

# plt.plot(t_real,w_stance_B/max(w_stance_B),Label="Stance B weight scaled")
# plt.plot(t_real,prob_stance_B,Label="Stance B prob")

# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("Stance")
# plt.title("Stance B - time")

# plt.figure()


# plt.plot(t_real,fA_z,Label="fA_z ")
# plt.plot(t_real,fB_z,Label="fB_z ")


# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("Forces")
# plt.title("Forces of swinging legs  - time")




# plt.figure()

# plt.plot(t_real[3025:3600],w_swing_A[3025:3600]/10000000,Label="Swing A weight scaled")
# plt.plot(t_real[3025:3600],prob_swing_A[3025:3600],Label="Swing A stable prob")
# plt.plot(t_real[3025:3600],fA_z[3025:3600]/10,Label="fA_z/10 ")
# # plt.plot(t_real[3025:3600],prob_stance_A[3025:3600],Label="Stance A stable prob")

# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("Swing")
# plt.title("Swing A  - time")

# plt.figure()

# plt.plot(t_real[3025:3600],w_swing_B[3025:3600]/10000000,Label="Swing B weight scaled")
# plt.plot(t_real[3025:3600],prob_swing_B[3025:3600],Label="Swing B prob")
# plt.plot(t_real[3025:3600],fB_z[3025:3600]/10,Label="fB_z/10 ")
# # plt.plot(t_real,prob_stance_B,Label="Stance B prob")

# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("Swing")
# plt.title("Swing B - time")


# plt.figure()
# # plt.plot(t_real,fB_z/max(fB_z),Label="fB_z normalized ")
# plt.plot(t_real[3025:3600],prob_stance_B[3025:3600],Label="Stance B prob")
# plt.plot(t_real[3025:3600],w_stance_B[3025:3600]/100,Label="Stance B weight scaled")
# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("Stance B")
# plt.title("Stance B  - time")

# plt.figure()
# # plt.plot(t_real,fA_z/max(fA_z),Label="fA_z normalized ")
# plt.plot(t_real[3025:3600],prob_stance_A[3025:3600],Label="Stance A prob")
# plt.plot(t_real[3025:3600],w_stance_A[3025:3600]/100,Label="Stance A weight scaled")
# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("Stance A")
# plt.title("Stance  - time")

fig1, axs = plt.subplots(2, 1)

axs[0].plot(t_real[3025:3600], prob_stance_A[3025:3600], label="Stable prob", color="darkslategrey", linestyle="dashed")
# axs[0].plot( t_real[3025:3600], w_stance_A[3025:3600]/100, label="Scaled weight", color="darkviolet")
axs[0].set_xlabel('Time (s)')
axs[0].set_ylabel('Stable Probability', color="darkslategrey")
# axs[0].legend(loc='upper right')
axs[0].tick_params(axis='y', labelcolor='darkslategrey')
axs[0].set_title("Front foot", loc='left', color='darkslategrey')

#twin axs[0]
ax2 = axs[0].twinx()
ax2.plot(t_real[3025:3600], w_stance_A[3025:3600], color='limegreen', label='Weights')
ax2.set_ylabel('Weights', color='limegreen')
ax2.tick_params(axis='y', labelcolor='limegreen')

axs[1].plot(t_real[3025:3600], prob_stance_B[3025:3600], label="Stable prob", color="midnightblue", linestyle="dashed")
# axs[1].plot(t_real[3025:3600], w_stance_B[3025:3600]/100, label="Scaled weight", color="b")
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Stable Probability', color="midnightblue")
# axs[1].legend(loc='upper right')
axs[1].tick_params(axis='y', labelcolor='midnightblue')
axs[1].set_title("Rear foot", loc='left', color="midnightblue")

#twin axs[1]
ax3 = axs[1].twinx()
ax3.plot(t_real[3025:3600], w_stance_B[3025:3600], color='blue', label='Weights')
ax3.set_ylabel('Weights', color='blue')
ax3.tick_params(axis='y', labelcolor='blue')

fig1.suptitle('Weights Adaptation Based on Stable Contact Probabilty')
############################################
#  (controller->t_real < 4.0) ?  (controller->t_real)/4.0*dp_cmd : dp_cmd 
vx_desired = []
vy_desired = []
vz_desired = []
dp_cmd = 0.6
for t in t_real:
    if (t < 4.0):
        vx_desired.append(t/4.0*dp_cmd)
    else:
        vx_desired.append(dp_cmd)
    vy_desired.append(0)
    vz_desired.append(0)
    

pc0_desired = []
# pc0_desired.append(0.0)
p_prev = 0.0
for v in vx_desired:
    pc0_desired.append(p_prev + 0.002*v )
    p_prev = p_prev + 0.002*v

fig2, axs2 = plt.subplots(3, 1)

axs2[0].plot(t_real, pc_0, label="Adapt.", color="mediumblue")
# axs2[0].plot(t_real_mpc, pc_0_mpc, label="MPC", color="mediumblue")
axs2[0].plot(t_real_slip, pc_0_slip, label="Without Adapt.", color="crimson")
axs2[0].plot(t_real_slip, pc0_desired, label="Desired", color="g")
axs2[0].set_xlabel('Time (s)')
axs2[0].set_ylabel('$Pos. x-axis(m)$')
axs2[0].axvspan(10.7, 33.10, color='grey', alpha=.2, label="Slippery surface")
axs2[0].set_xlim(0,33.1)
# axs2[0].legend(loc='bottom')
axs2[0].legend(loc='upper center', bbox_to_anchor=(0.5, 1.35),
          ncol=4, fancybox=True, shadow=True)


axs2[1].plot(t_real, pc_1, label="Adapt.", color="mediumblue")
axs2[1].plot(t_real_slip, pc_1_slip, label="Without Adapt.", color="crimson")
axs2[1].plot(t_real_slip, np.zeros(len(pc0_desired)), label="Desired", color="g")
# axs2[1].axhline(0.0, color='g', label="Desired")
axs2[1].set_xlabel('Time (s)')
axs2[1].set_ylabel('$Pos. y-axis(m)$')
axs2[1].axvspan(10.7, 33.10, color='grey', alpha=.2, label="Slippery surface")
axs2[1].set_xlim(0,33.1)
# axs2[1].legend(loc='bottom')

axs2[2].plot(t_real, pc_2, label="Adapt.", color="mediumblue")
axs2[2].plot(t_real_slip, pc_2_slip, label="Without Adapt.", color="crimson")
axs2[2].plot(t_real_slip, 0.384*np.ones(len(pc0_desired)), label="Desired", color="g")
# axs2[2].axhline(0.384, color='g', label="Desired")
axs2[2].set_xlabel('Time (s)')
axs2[2].set_ylabel('$Pos. z-axis(m)$')
axs2[2].axvspan(10.7, 33.10, color='grey', alpha=.2, label="Slippery surface")
axs2[2].set_xlim(0,33.1)
# axs2[2].legend(loc='bottom')

fig2.suptitle('CoM World Position')

# fig, axs3 = plt.subplots(3, 1)

# axs3[0].plot(t_real, vx, label="PCE adapt", color="mediumblue")
# axs3[0].plot(t_real_slip, vx_slip, label="Without PCE adapt", color="crimson")
# axs3[0].plot(t_real, vx_desired, label="Desired", color="crimson")
# axs3[0].set_ylabel('X vel')
# axs3[0].legend(loc='upper left')

# axs3[1].plot(t_real, vy, label="PCE adapt", color="mediumblue")
# axs3[1].plot(t_real_slip, vy_slip, label="Without PCE adapt", color="crimson")
# axs3[1].set_xlabel('Time (s)')
# axs3[1].set_ylabel('Y vel')
# axs3[1].legend(loc='upper left')

# axs3[2].plot(t_real, vz, label="PCE adapt", color="mediumblue")
# axs3[2].plot(t_real_slip, vz_slip, label="Without PCE adapt", color="crimson")
# axs3[2].axhline(0.0, color='black')

# axs3[2].set_ylabel('Z vel')
# axs3[2].legend(loc='upper left')

fig3, axs3 = plt.subplots(3, 1)

axs3[0].plot(t_real, er_vel_x, label="Adapt.", color="mediumblue")
axs3[0].plot(t_real_slip, er_vel_x_slip, label="Without Adapt.", color="crimson")
axs3[0].axhline(0.0, color='g', label="Desired-Zero")
axs3[0].axvspan(10.7, 33.10, color='grey', alpha=.2, label="Slippery surface")
axs3[0].set_xlim(0,33.1)
axs3[0].set_xlabel('Time (s)')
axs3[0].set_ylabel('$x-axis$(m/s)')
# axs3[0].legend(loc='outside upper center')
axs3[0].legend(loc='upper center', bbox_to_anchor=(0.5, 1.35),
          ncol=4, fancybox=True, shadow=True)

# axs3[0].legend(loc='bottom right')

axs3[1].plot(t_real, er_vel_y, label="Adapt.", color="mediumblue")
axs3[1].plot(t_real_slip, er_vel_y_slip, label="Without Adapt.", color="crimson")
axs3[1].set_xlabel('Time (s)')
axs3[1].set_ylabel('$y-axis$(m/s)')
axs3[1].axvspan(10.7, 33.10, color='grey', alpha=.2, label="Slippery surface")
axs3[1].set_xlim(0,33.1)
axs3[1].axhline(0.0, color='g', label="Desired-Zero")
# axs3[1].legend(loc='bottom right')

axs3[2].plot(t_real, er_vel_z, label="Adapt.", color="mediumblue")
axs3[2].plot(t_real_slip, er_vel_z_slip, label="Without Adapt.", color="crimson")
axs3[2].axhline(0.0, color='g', label="Desired-Zero")
axs3[2].axvspan(10.7, 33.10, color='grey', alpha=.2, label="Slippery surface")
axs3[2].set_xlim(0,33.1)
axs3[2].set_xlabel('Time (s)')
axs3[2].set_ylabel('$z-axis$(m/s)')
# axs3[2].legend(loc='bottom right')

fig3.suptitle("CoM Velocity Error")

print(np.where(t_real==10.7))
print(np.std(pc_0[np.where(t_real<10.7)]))



print("STD VEL adaptation (x-y-z) axis")
print(np.std(vx[0:5349]))
print(np.std(vy[0:5349]))
print(np.std(vz[0:5349]))
print("STD VEL without adaptation (x-y-z) axis")
print(np.std(vx_slip[0:5349]))
print(np.std(vy_slip[0:5349]))
print(np.std(vz_slip[0:5349]))


print("STD VEL adaptation (x-y-z) axis: Slippery surface")
print(np.std(vx[5349:]))
print(np.std(vy[5349:]))
print(np.std(vz[5349:]))
print("STD VEL without adaptation (x-y-z) axis: Slippery surface")
print(np.std(vx_slip[5349:]))
print(np.std(vy_slip[5349:]))
print(np.std(vz_slip[5349:]))


from sklearn.metrics import mean_absolute_error
print("MAE VEL adaptation (x-y-z) axis: Total")
print(mean_absolute_error(vx,vx_desired))
print(mean_absolute_error(vy,np.zeros(len(vx_desired))))
print(mean_absolute_error(vz,np.zeros(len(vx_desired))))

print("MAE VEL without adaptation (x-y-z) axis: Total")
print(mean_absolute_error(vx_slip,vx_desired))
print(mean_absolute_error(vy_slip,np.zeros(len(vx_desired))))
print(mean_absolute_error(vz_slip,np.zeros(len(vx_desired))))

print("MAE VEL adaptation (x-y-z) axis")
print(mean_absolute_error(vx[0:5349],vx_desired[0:5349]))
print(mean_absolute_error(vy[0:5349],np.zeros(len(vx_desired[0:5349]))))
print(mean_absolute_error(vz[0:5349],np.zeros(len(vx_desired[0:5349]))))

print("MAE VEL without adaptation (x-y-z) axis")
print(mean_absolute_error(vx_slip[0:5349],vx_desired[0:5349]))
print(mean_absolute_error(vy_slip[0:5349],np.zeros(len(vx_desired[0:5349]))))
print(mean_absolute_error(vz_slip[0:5349],np.zeros(len(vx_desired[0:5349]))))


print("MAE VEL adaptation (x-y-z) axis: Slippery surface")
print(mean_absolute_error(vx[5349:],vx_desired[5349:]))
print(mean_absolute_error(vy[5349:],np.zeros(len(vx_desired[5349:]))))
print(mean_absolute_error(vz[5349:],np.zeros(len(vx_desired[5349:]))))
print("MAE VEL without adaptation (x-y-z) axis: Slippery surface")
print(mean_absolute_error(vx_slip[5349:],vx_desired[5349:]))
print(mean_absolute_error(vy_slip[5349:],np.zeros(len(vx_desired[5349:]))))
print(mean_absolute_error(vz_slip[5349:],np.zeros(len(vx_desired[5349:]))))




plt.show()
plt.waitforbuttonpress(0) # this will wait for indefinite timeplt.close(fig)
plt.close('all')


#!/usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
import os

end1 = 16550
end2 = 1655
os.chdir("../../data/")

data = np.genfromtxt("./data_OTD_adapt.csv", delimiter=" ", skip_header=1)
data_slip = np.genfromtxt("./data_OTD_withoutadapt.csv", delimiter=" ", skip_header=1)
num = np.min( [(len(data)), (len(data_slip))])

data = data[0:num,:]
data_slip = data_slip[0:num,:]

data_RL = np.genfromtxt("./RL_data.csv", delimiter=" ", skip_header=1)
# times
t_real_RL = data_RL[:,0]*0.02/0.004

# CoM current pos
pc_0_RL = data_RL[:,1]
pc_1_RL = data_RL[:,2]
pc_2_RL = data_RL[:,3]

# CoM desired pos
vx_RL = data_RL[:,4]
vy_RL = data_RL[:,5]
vz_RL = data_RL[:,6]

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

plt.plot(t_real[3025:3200],w0[3025:3200],Label="$w_0$")
plt.plot(t_real[3025:3200],w1[3025:3200],Label="$w_1$")
plt.plot(t_real[3025:3200],w2[3025:3200],Label="$w_2$")
plt.plot(t_real[3025:3200],w3[3025:3200],Label="$w_3$")

plt.axvline(x=6.202, color='k', linestyle='dashed', Label="$t_D$")
plt.axvline(x=6.204, color='k', linestyle='dashed')
plt.axvline(x=6.376, color='k', linestyle='dashed')
plt.axvline(x=6.378, color='k', linestyle='dashed')

plt.fill_between(t_real[3025:3200], 10000000, where=w3[3025:3200] > 110, facecolor='red', alpha=.2)
plt.fill_between(t_real[3025:3200], 10000000, where=w1[3025:3200] > 110, facecolor='gold', alpha=.1)

plt.legend(loc='upper left')
plt.xlabel("Time (s)")
plt.ylabel("Weights $x-axis$")
plt.title("Swinging weights")


fig1, axs = plt.subplots(2, 1)
axs[0].plot(t_real[3025:3600], prob_stance_A[3025:3600], label="Stable prob", color="darkslategrey", linestyle="dashed")
axs[0].set_xlabel('Time (s)')
axs[0].set_ylabel('Stable Probability', color="darkslategrey")
axs[0].tick_params(axis='y', labelcolor='darkslategrey')
axs[0].set_title("Front foot", loc='left', color='darkslategrey')
axs[0].axvspan(6.071, 6.243, color='gold', alpha=.1, label="L")
axs[0].axvspan(6.243, 6.419, color='r', alpha=.2, label="R")
axs[0].axvspan(6.419, 6.5905, color='gold', alpha=.1)
axs[0].axvspan(6.5905, 6.7670, color='r', alpha=.2)
axs[0].axvspan(6.7670, 6.9385, color='gold', alpha=.1)
axs[0].axvspan(6.9385, 7.115, color='r', alpha=.2)
axs[0].axvspan(7.115, 7.200, color='gold', alpha=.1)

ax2 = axs[0].twinx()
ax2.plot(t_real[3025:3600], w_stance_A[3025:3600], color='limegreen', label='Weights')
ax2.set_ylabel('Weights', color='limegreen')
ax2.tick_params(axis='y', labelcolor='limegreen')

axs[1].plot(t_real[3025:3600], prob_stance_B[3025:3600], label="Stable prob", color="midnightblue", linestyle="dashed")
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Stable Probability', color="midnightblue")
axs[1].tick_params(axis='y', labelcolor='midnightblue')
axs[1].set_title("Rear foot", loc='left', color="midnightblue")
axs[1].axvspan(6.071, 6.243, color='gold', alpha=.1, label="R")
axs[1].axvspan(6.243, 6.419, color='r', alpha=.2, label="L")
axs[1].axvspan(6.419, 6.5905, color='gold', alpha=.1, label="R")
axs[1].axvspan(6.5905, 6.7670, color='r', alpha=.2, label="L")
axs[1].axvspan(6.7670, 6.9385, color='gold', alpha=.1, label="R")
axs[1].axvspan(6.9385, 7.115, color='r', alpha=.2, label="L")
axs[1].axvspan(7.115, 7.200, color='gold', alpha=.1, label="R")

ax3 = axs[1].twinx()
ax3.plot(t_real[3025:3600], w_stance_B[3025:3600], color='blue', label='Weights')
ax3.set_ylabel('Weights', color='blue')
ax3.tick_params(axis='y', labelcolor='blue')

fig1.suptitle('Weights Adaptation Based on Stable Contact Probabilty')
vx_desired = []
vy_desired = []
vz_desired = []
dp_cmd = 0.68
dp_cmd_y = 0.0
for t in t_real:
    if (t < 4.0):
        vx_desired.append(t/4.0*dp_cmd)
        vy_desired.append(t/4.0*dp_cmd_y)

    else:
        vx_desired.append(dp_cmd)
        vy_desired.append(dp_cmd_y)

    vz_desired.append(0)
    

pc0_desired = []
pc1_desired = []
p_prev = 0.0
for v in vx_desired:
    pc0_desired.append(p_prev + 0.002*v )
    p_prev = p_prev + 0.002*v

p_prev = 0.0
for v in vy_desired:
    pc1_desired.append(p_prev + 0.002*v )
    p_prev = p_prev + 0.002*v

fig2, axs2 = plt.subplots(3, 1)

axs2[0].plot(t_real, pc_0, label="Proposed Adapt.", color="mediumblue")
axs2[0].plot(t_real_slip, pc_0_slip, label="Proposed Without Adapt.", color="crimson")
axs2[0].plot(t_real_RL, pc_0_RL, label="Compared-RL", color="purple")
axs2[0].plot(t_real_slip, pc0_desired, label="Desired", color="g")
axs2[0].set_xlabel('Time (s)')
axs2[0].set_ylabel('$Pos. x-axis(m)$')
axs2[0].axvspan(10.0, 33.10, color='grey', alpha=.2, label="Slippery surface")
axs2[0].set_xlim(0,33.1)
axs2[0].legend(loc='upper center', bbox_to_anchor=(0.5, 1.35),
          ncol=4, fancybox=True, shadow=True)


axs2[1].plot(t_real, pc_1, label="Proposed Adapt.", color="mediumblue")
axs2[1].plot(t_real_slip, pc_1_slip, label="Proposed Without Adapt.", color="crimson")
axs2[1].plot(t_real_RL, pc_1_RL, label="Compared-RL", color="purple")
axs2[1].plot(t_real_slip, pc1_desired, label="Desired", color="g")
axs2[1].set_xlabel('Time (s)')
axs2[1].set_ylabel('$Pos. y-axis(m)$')
axs2[1].axvspan(10.0, 33.10, color='grey', alpha=.2, label="Slippery surface")
axs2[1].set_xlim(0,33.1)

axs2[2].plot(t_real, pc_2, label="Proposed Adapt.", color="mediumblue")
axs2[2].plot(t_real_slip, pc_2_slip, label="Proposed Without Adapt.", color="crimson")
axs2[2].plot(t_real_RL, pc_2_RL, label="Compared-RL", color="purple")
axs2[2].plot(t_real_slip, 0.384*np.ones(len(pc0_desired)), label="Desired", color="g")
axs2[2].set_xlabel('Time (s)')
axs2[2].set_ylabel('$Pos. z-axis(m)$')
axs2[2].axvspan(10.0, 33.10, color='grey', alpha=.2, label="Slippery surface")
axs2[2].set_xlim(0,33.1)

fig2.suptitle('CoM World Position')


fig3, axs3 = plt.subplots(3, 1)

axs3[0].plot(t_real, er_vel_x, label="Proposed Adapt.", color="mediumblue")
axs3[0].plot(t_real_slip, er_vel_x_slip, label="Proposed Without Adapt.", color="crimson")
axs3[0].plot(t_real_RL, vx_RL - 0.68*np.ones(len(t_real_RL)), label="Compared-RL", color="purple")
axs3[0].axhline(0.0, color='g', label="Desired-Zero")
axs3[0].axvspan(10.0, 33.10, color='grey', alpha=.2, label="Slippery surface")
axs3[0].set_xlim(0,33.1)
axs3[0].set_xlabel('Time (s)')
axs3[0].set_ylabel('$x-axis$(m/s)')
axs3[0].legend(loc='upper center', bbox_to_anchor=(0.5, 1.35),
          ncol=4, fancybox=True, shadow=True)


axs3[1].plot(t_real, er_vel_y, label="Proposed Adapt.", color="mediumblue")
axs3[1].plot(t_real_slip, er_vel_y_slip, label="Proposed Without Adapt.", color="crimson")
axs3[1].plot(t_real_RL, vy_RL, label="Compared-RL", color="purple")
axs3[1].axhline(0.0, color='g', label="Desired-Zero")
axs3[1].set_xlabel('Time (s)')
axs3[1].set_ylabel('$y-axis$(m/s)')
axs3[1].axvspan(10.0, 33.10, color='grey', alpha=.2, label="Slippery surface")
axs3[1].set_xlim(0,33.1)

axs3[2].plot(t_real, er_vel_z, label="Proposed Adapt.", color="mediumblue")
axs3[2].plot(t_real_slip, er_vel_z_slip, label="Proposed Without Adapt.", color="crimson")
axs3[2].plot(t_real_RL, vz_RL, label="Compared-RL", color="purple")
axs3[2].axhline(0.0, color='g', label="Desired-Zero")
axs3[2].axvspan(10.0, 33.10, color='grey', alpha=.2, label="Slippery surface")
axs3[2].set_xlim(0,33.1)
axs3[2].set_xlabel('Time (s)')
axs3[2].set_ylabel('$z-axis$(m/s)')

fig3.suptitle("CoM Velocity Error")

print("median VEL adaptation (x-y-z) axis")
print(np.median(vx[2000:5000]))
print(np.median(vy[0:5000]))
print(np.median(vz[0:5000]))

print("median VEL RL (x-y-z) axis")
print(np.median(vx_RL[0:499]))
print(np.median(vy_RL[0:499]))
print(np.median(vz_RL[0:499]))


print("median VEL adaptation (x-y-z) axis: Slippery surface")
print(np.median(vx[5000:end1]))
print(np.median(vy[5000:end1]))
print(np.median(vz[5000:end1]))

print("median VEL RL (x-y-z) axis: Slippery surface")
print(np.median(vx_RL[499:end2]))
print(np.median(vy_RL[499:end2]))
print(np.median(vz_RL[499:end2]))


print("STD VEL adaptation (x-y-z) axis")
print(np.std(vx[2000:5000]))
print(np.std(vy[0:5000]))
print(np.std(vz[0:5000]))

print("STD VEL RL (x-y-z) axis")
print(np.std(vx_RL[0:499]))
print(np.std(vy_RL[0:499]))
print(np.std(vz_RL[0:499]))


print("STD VEL adaptation (x-y-z) axis: Slippery surface")
print(np.std(vx[5000:end1]))
print(np.std(vy[5000:end1]))
print(np.std(vz[5000:end1]))

print("STD VEL RL (x-y-z) axis: Slippery surface")
print(np.std(vx_RL[499:end2]))
print(np.std(vy_RL[499:end2]))
print(np.std(vz_RL[499:end2]))

###############################################################################

plt.show()
plt.waitforbuttonpress(0) # this will wait for indefinite timeplt.close(fig)
plt.close('all')


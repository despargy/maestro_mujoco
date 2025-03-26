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


plt.figure()

plt.plot(t_real,w0,label="$w_0$")
plt.plot(t_real,w1,label="$w_1$")
plt.plot(t_real,w2,label="$w_2$")
plt.plot(t_real,w3,label="$w_3$")

# plt.axvline(x=6.202, color='k', linestyle='dashed', Label="$t_D$")
# plt.axvline(x=6.204, color='k', linestyle='dashed')
# plt.axvline(x=6.376, color='k', linestyle='dashed')
# plt.axvline(x=6.378, color='k', linestyle='dashed')

plt.fill_between(t_real, 10000000, where=w3 > 110, facecolor='red', alpha=.2)
plt.fill_between(t_real, 10000000, where=w1 > 110, facecolor='gold', alpha=.1)
plt.ylim(50,200)
plt.legend(loc='upper left')
plt.xlabel("Time (s)")
plt.ylabel("Weights $x-axis$")
plt.title("Weights")


fig1, axs = plt.subplots(2, 1)
axs[0].plot(t_real, prob_stance_A, label="Stable prob", color="darkslategrey", linestyle="dashed")
axs[0].set_xlabel('Time (s)')
axs[0].set_ylabel('Stable Probability', color="darkslategrey")
axs[0].tick_params(axis='y', labelcolor='darkslategrey')
axs[0].set_title("Front foot", loc='left', color='darkslategrey')

ax2 = axs[0].twinx()
ax2.plot(t_real, w_stance_A, color='limegreen', label='Weights')
ax2.set_ylabel('Weights', color='limegreen')
ax2.tick_params(axis='y', labelcolor='limegreen')

axs[1].plot(t_real, prob_stance_B, label="Stable prob", color="midnightblue", linestyle="dashed")
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Stable Probability', color="midnightblue")
axs[1].tick_params(axis='y', labelcolor='midnightblue')
axs[1].set_title("Rear foot", loc='left', color="midnightblue")

ax3 = axs[1].twinx()
ax3.plot(t_real, w_stance_B, color='blue', label='Weights')
ax3.set_ylabel('Weights', color='blue')
ax3.tick_params(axis='y', labelcolor='blue')

# fig1.suptitle('Weights Adaptation Based on Stable Contact Probabilty')
# vx_desired = []
# vy_desired = []
# vz_desired = []
# dp_cmd = 0.68
# dp_cmd_y = 0.0
# for t in t_real:
#     if (t < 4.0):
#         vx_desired.append(t/4.0*dp_cmd)
#         vy_desired.append(t/4.0*dp_cmd_y)

#     else:
#         vx_desired.append(dp_cmd)
#         vy_desired.append(dp_cmd_y)

#     vz_desired.append(0)
    

# pc0_desired = []
# pc1_desired = []
# p_prev = 0.0
# for v in vx_desired:
#     pc0_desired.append(p_prev + 0.002*v )
#     p_prev = p_prev + 0.002*v

# p_prev = 0.0
# for v in vy_desired:
#     pc1_desired.append(p_prev + 0.002*v )
#     p_prev = p_prev + 0.002*v

# fig2, axs2 = plt.subplots(3, 1)

# axs2[0].plot(t_real, pc_0, label="Proposed Adapt.", color="mediumblue")
# axs2[0].set_xlabel('Time (s)')
# axs2[0].set_ylabel('$Pos. x-axis(m)$')
# axs2[0].axvspan(10.0, 33.10, color='grey', alpha=.2, label="Slippery surface")
# axs2[0].set_xlim(0,33.1)
# axs2[0].legend(loc='upper center', bbox_to_anchor=(0.5, 1.35),
#           ncol=4, fancybox=True, shadow=True)


# axs2[1].plot(t_real, pc_1, label="Proposed Adapt.", color="mediumblue")
# axs2[1].set_xlabel('Time (s)')
# axs2[1].set_ylabel('$Pos. y-axis(m)$')
# axs2[1].axvspan(10.0, 33.10, color='grey', alpha=.2, label="Slippery surface")
# axs2[1].set_xlim(0,33.1)

# axs2[2].plot(t_real, pc_2, label="Proposed Adapt.", color="mediumblue")
# axs2[2].set_xlabel('Time (s)')
# axs2[2].set_ylabel('$Pos. z-axis(m)$')
# axs2[2].axvspan(10.0, 33.10, color='grey', alpha=.2, label="Slippery surface")
# axs2[2].set_xlim(0,33.1)

# fig2.suptitle('CoM World Position')


# fig3, axs3 = plt.subplots(3, 1)

# axs3[0].plot(t_real, er_vel_x, label="Proposed Adapt.", color="mediumblue")
# axs3[0].axhline(0.0, color='g', label="Desired-Zero")
# axs3[0].axvspan(10.0, 33.10, color='grey', alpha=.2, label="Slippery surface")
# axs3[0].set_xlim(0,33.1)
# axs3[0].set_xlabel('Time (s)')
# axs3[0].set_ylabel('$x-axis$(m/s)')
# axs3[0].legend(loc='upper center', bbox_to_anchor=(0.5, 1.35),
#           ncol=4, fancybox=True, shadow=True)


# axs3[1].plot(t_real, er_vel_y, label="Proposed Adapt.", color="mediumblue")
# axs3[1].axhline(0.0, color='g', label="Desired-Zero")
# axs3[1].set_xlabel('Time (s)')
# axs3[1].set_ylabel('$y-axis$(m/s)')
# axs3[1].axvspan(10.0, 33.10, color='grey', alpha=.2, label="Slippery surface")
# axs3[1].set_xlim(0,33.1)

# axs3[2].plot(t_real, er_vel_z, label="Proposed Adapt.", color="mediumblue")
# axs3[2].axhline(0.0, color='g', label="Desired-Zero")
# axs3[2].axvspan(10.0, 33.10, color='grey', alpha=.2, label="Slippery surface")
# axs3[2].set_xlim(0,33.1)
# axs3[2].set_xlabel('Time (s)')
# axs3[2].set_ylabel('$z-axis$(m/s)')

# fig3.suptitle("CoM Velocity Error")

###############################################################################

plt.show()
plt.waitforbuttonpress(0) # this will wait for indefinite timeplt.close(fig)
plt.close('all')


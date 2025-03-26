#!/usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation as animation
from mpl_toolkits import mplot3d
import os


os.chdir("../../data/")

data = np.genfromtxt("./data.csv", delimiter=" ", skip_header=1)
# data=data[0:100,:]
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



fig1, axs = plt.subplots(2, 1)
ax1 = axs[0]
ax1_twin = ax1.twinx()

ax2 = axs[1]
ax2_twin = ax2.twinx()

line1, = ax1.plot([], [], label="Stable prob", color="darkslategrey", linestyle="dashed")
line2, = ax1_twin.plot([], [], color='limegreen', label='Weights')

line3, = ax2.plot([], [], label="Stable prob", color="midnightblue", linestyle="dashed")
line4, = ax2_twin.plot([], [], color='blue', label='Weights')

# Labels and Titles
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Stable Probability', color="darkslategrey")
ax1_twin.set_ylabel('Weights', color='limegreen')
ax1.set_title("Front foot", loc='left', color='darkslategrey')

ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Stable Probability', color="midnightblue")
ax2_twin.set_ylabel('Weights', color='blue')
ax2.set_title("Rear foot", loc='left', color="midnightblue")

# def update(frame):
#     # Update front foot plot (both y-axes)
#     line1.set_data(t_real[:frame], prob_stance_A[:frame])
#     line2.set_data(t_real[:frame], w_stance_A[:frame])

#     # Update rear foot plot (both y-axes)
#     line3.set_data(t_real[:frame], prob_stance_B[:frame])
#     line4.set_data(t_real[:frame], w_stance_B[:frame])

#     # Rescale both y-axes dynamically
#     ax1.relim()
#     ax1.autoscale_view()
#     ax1_twin.relim()
#     ax1_twin.autoscale_view()

#     ax2.relim()
#     ax2.autoscale_view()
#     ax2_twin.relim()
#     ax2_twin.autoscale_view()

#     return line1, line2, line3, line4

########################## Sliding ######################

window_duration = 0.2  # Show last 3 seconds
# Update function for animation (show last 3 sec)
def update(frame):
    t_max = t_real[frame]  # Current time
    t_min = max(0, t_max - window_duration)  # Show last 3 sec

    # Find indices for the sliding window
    mask = (t_real >= t_min) & (t_real <= t_max)

    # Update front foot plot
    line1.set_data(t_real[mask], prob_stance_A[mask])
    line2.set_data(t_real[mask], w_stance_A[mask])

    # Update rear foot plot
    line3.set_data(t_real[mask], prob_stance_B[mask])
    line4.set_data(t_real[mask], w_stance_B[mask])

    # Adjust x-axis limits dynamically
    ax1.set_xlim(t_min, t_max)
    ax2.set_xlim(t_min, t_max)

    # Rescale y-axes
    ax1.relim()
    ax1.autoscale_view()
    ax1_twin.relim()
    ax1_twin.autoscale_view()

    ax2.relim()
    ax2.autoscale_view()
    ax2_twin.relim()
    ax2_twin.autoscale_view()

    return line1, line2, line3, line4


# Create the animation
ani = animation.FuncAnimation(fig1, update, frames=len(t_real), interval=1, blit=False)

# Save as GIF
# ani.save('/home/despinar/mujoco_ws/maestro_mujoco/src/create_plots/real_time_plot.gif', writer=animation.PillowWriter(fps=500))

plt.show()

###############################################################################

# plt.show()
# plt.waitforbuttonpress(0) # this will wait for indefinite timeplt.close(fig)
# plt.close('all')


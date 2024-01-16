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
ev_0 = data[:,1]
ev_1 = data[:,2]
ev_2 = data[:,3]
# Weights for each tip (x-axis)
w0 = data[:,4]
w1 = data[:,5]
w2 = data[:,6]
w3 = data[:,7]
# Each swinging desired tip pos
tipd_a0 = data[:,8]
tipd_a1 = data[:,9]
tipd_a2 = data[:,10]

tipd_b0 = data[:,11]
tipd_b1 = data[:,12]
tipd_b2 = data[:,13]

# CoM pos
x1_d = data[:,14]
y2_d = data[:,15]
z3_d = data[:,16]

# Each swinging tip forces transformed
f = data[:,17]


plt.figure()
plt.plot(t_real,ev_0,Label="ev x")
plt.plot(t_real,ev_1,Label="ev y")
plt.plot(t_real,ev_2,Label="ev z")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("Vel error")
plt.title("Vel error  - time")

plt.figure()

plt.plot(t_real,w0,Label="w0 x")
plt.plot(t_real,w1,Label="w1 x")
plt.plot(t_real,w2,Label="w2 x")
plt.plot(t_real,w3,Label="w3 x")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("Weights position")
plt.title("Weights  - time")

plt.figure()


plt.plot(t_real,tipd_a0,Label="Tip A x ")
plt.plot(t_real,tipd_a1,Label="Tip A y ")
plt.plot(t_real,tipd_a2,Label="Tip A z ")

plt.plot(t_real,tipd_b0,Label="Tip B x")
plt.plot(t_real,tipd_b1,Label="Tip B y")
plt.plot(t_real,tipd_b2,Label="Tip B z")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("Tip pos")
plt.title("Tips  - time")

plt.figure()


plt.plot(t_real,w0/10000,Label="weight")
# plt.plot(t_real,p0d,Label="p0d")
# plt.plot(t_real,z3,Label="z actual")
plt.axvline(0.02, color='k',label='t0 swing')
plt.axvline(0.05, color='b',label='t half swing swinging')
plt.axvline(2.11, color='r',label='end swinging')

plt.legend()
plt.xlabel("t_real")
plt.ylabel("CoM pos")
plt.title("CoM  - time")


plt.figure()
plt.plot(t_real,f,Label="f norm")
# plt.plot(t_real,f1,Label="f 1")
# plt.plot(t_real,f2,Label="f 2")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("Tip forces")
plt.title("Forces  - time")

plt.show()
plt.waitforbuttonpress(0) # this will wait for indefinite timeplt.close(fig)
plt.close('all')


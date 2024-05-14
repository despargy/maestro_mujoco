#!/usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
import os

os.chdir("../../data/")
data = np.genfromtxt("./data_0.9.csv", delimiter=" ", skip_header=1)
data = data[0:9000,:]

data_slip = np.genfromtxt("./data_0.6.csv", delimiter=" ", skip_header=1)
# data_slip = np.genfromtxt("./data_compare_long.csv", delimiter=" ", skip_header=1)
data_slip = data_slip[0:9000,:]



# times
t_real = data[:,0]
# Each swinging tip pos
Vcx = data[:,1]
Vcy = data[:,2]
pcz = data[:,3]
# Weights for each tip (x-axis)
w0 = data[:,4]
w1 = data[:,5]
w2 = data[:,6]
w3 = data[:,7]

# Ori error
eo_0 = data[:,8]
eo_1 = data[:,9]
eo_2 = data[:,10]
#Vel error
ev_0 = data[:,11]
ev_1 = data[:,12]
ev_2 = data[:,13]
# tau norm
tau_0 = data[:,14]
tau_1 = data[:,15]
tau_2 = data[:,16]
tau_3 = data[:,17]

######################### SLIP #############################
# Each swinging tip pos
slip_Vcx = data_slip[:,1]
slip_Vcy = data_slip[:,2]
slip_pcz = data_slip[:,3]
# Weights for each tip (x-axis)
slip_w0 = data_slip[:,4]
slip_w1 = data_slip[:,5]
slip_w2 = data_slip[:,6]
slip_w3 = data_slip[:,7]

# Ori error
slip_eo_0 = data_slip[:,8]
slip_eo_1 = data_slip[:,9]
slip_eo_2 = data_slip[:,10]
#Vel error
slip_ev_0 = data_slip[:,11]
slip_ev_1 = data_slip[:,12]
slip_ev_2 = data_slip[:,13]
# tau norm
slip_tau_0 = data_slip[:,14]
slip_tau_1 = data_slip[:,15]
slip_tau_2 = data_slip[:,16]
slip_tau_3 = data_slip[:,17]
#########################    #############################

plt.figure()

plt.plot(t_real,w0,Label="w0")
plt.plot(t_real,w1,Label="w1")
plt.plot(t_real,w2,Label="w2")
plt.plot(t_real,w3,Label="w3")

plt.plot(t_real,slip_w0,Label="SLIP w0")
plt.plot(t_real,slip_w1,Label="SLIP w1")
plt.plot(t_real,slip_w2,Label="SLIP w2")
plt.plot(t_real,slip_w3,Label="SLIP w3")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("Weights")
plt.title("Weights  - time")

plt.figure()
plt.plot(t_real,Vcx,Label="Vcx")
plt.plot(t_real,slip_Vcx,Label="Slip Vcx")
plt.axhline(y=0.6, color='r', linestyle='-', Label="desired vel")
plt.axhline(y=0.9, color='r', linestyle='-', Label="desired vel")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("Vcx")
plt.title("Vcx  - time")


plt.figure()
plt.plot(t_real,Vcy,Label="Vcy")
plt.plot(t_real,slip_Vcy,Label="Slip Vcy")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("Vcy")
plt.title("Vcy  - time")

plt.figure()
plt.plot(t_real,pcz,Label="pcz")
plt.plot(t_real,slip_pcz,Label="Slip pcz")
plt.axhline(y=0.4, color='g', linestyle='-', Label="desired z")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("pcz")
plt.title("pcz  - time")


plt.figure()
plt.plot(t_real,eo_0,Label="eo_0")
plt.plot(t_real,slip_eo_0,Label="Slip eo_0")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("eo_0")
plt.title("eo_0  - time")

plt.figure()
plt.plot(t_real,eo_1,Label="eo_1")
plt.plot(t_real,slip_eo_1,Label="Slip eo_1")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("eo_1")
plt.title("eo_1  - time")


plt.figure()
plt.plot(t_real,eo_2,Label="eo_2")
plt.plot(t_real,slip_eo_2,Label="Slip eo_2")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("eo_2")
plt.title("eo_2  - time")



plt.figure()
plt.plot(t_real,ev_0,Label="ev_0")
plt.plot(t_real,slip_ev_0,Label="Slip ev_0")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("ev_0")
plt.title("ev_0  - time")

plt.figure()
plt.plot(t_real,ev_1,Label="ev_1")
plt.plot(t_real,slip_ev_1,Label="Slip ev_1")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("ev_1")
plt.title("ev_1  - time")


plt.figure()
plt.plot(t_real,ev_2,Label="ev_2")
plt.plot(t_real,slip_ev_2,Label="Slip ev_2")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("ev_2")
plt.title("ev_2  - time")


plt.figure()
plt.plot(t_real,tau_0,Label="tau_0")
plt.plot(t_real,slip_tau_0,Label="Slip tau_0")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("tau_0")
plt.title("tau_0  - time")



plt.figure()
plt.plot(t_real,tau_1,Label="tau_1")
plt.plot(t_real,slip_tau_1,Label="Slip tau_1")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("tau_1")
plt.title("tau_1  - time")


plt.figure()
plt.plot(t_real,tau_2,Label="tau_2")
plt.plot(t_real,slip_tau_2,Label="Slip tau_2")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("tau_2")
plt.title("tau_2  - time")


plt.figure()
plt.plot(t_real,tau_3,Label="tau_3")
plt.plot(t_real,slip_tau_3,Label="Slip tau_3")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("tau_3")
plt.title("tau_3  - time")


plt.figure()
plt.plot(t_real,tau_0,Label="tau_0")
plt.plot(t_real,tau_1,Label="tau_1")
plt.plot(t_real,tau_2,Label="tau_2")
plt.plot(t_real,tau_3,Label="tau_3")

plt.legend()
plt.xlabel("t_real")
plt.ylabel("Tau")
plt.title("Tau  - time")


plt.show()
plt.waitforbuttonpress(0) # this will wait for indefinite timeplt.close(fig)
plt.close('all')


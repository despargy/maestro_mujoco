#!/usr/bin/env python3
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
# Import math Library
import math 

data = np.genfromtxt("/home/despinar/mujoco_ws/maestro_mujoco/data/data.csv", delimiter=" ", skip_header=1)

# times
t_real = data[:,0]


ep0 = data[:,1]
ep1 = data[:,2]
ep2 = data[:,3]

plt.figure()
plt.plot(t_real,ep0,Label="p error x")
plt.plot(t_real,ep1,Label="p error y")
plt.plot(t_real,ep2,Label="p error z")
plt.legend()
plt.xlabel("t_real")
plt.ylabel("Error position")
plt.title("Error  - time")


plt.show()
plt.waitforbuttonpress(0) # this will wait for indefinite timeplt.close(fig)
plt.close('all')


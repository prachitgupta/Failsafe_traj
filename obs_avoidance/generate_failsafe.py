#!/usr/bin/env python3
from operators import Y,occ, Yinv
from ref import Ref_path
from intended_arbitary import intended_traj
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
from obstacle import obstacles,ego
from invariably_safe_set import *
from initial_state import Initial_state
from long_prototype import optimize_longitudinal_trajectory
from shapely.geometry import Point, Polygon

T = Ref_path([0, 10], [100, 10], 5)
Tin = intended_traj([(1, 10), (4, 15), (30, 11), (50, 10)])
x_interpolated, y_interpolated = Tin.generate_path()
x0 = Initial_state()
status = x0.initial_state(Tin)

if(status == "verified"):
    print("execute intended trajectory")
    
elif(status == "boom"):
    print("don't bother it collided")

else:
    TTR,T_safe, initial_state_long , initial_state_lat = status
    T_safe_x,T_safe_y = status[1] 

xref, yref = T.ref_trajectory()
time = np.linspace(0,T.T,100)
plt.figure(figsize=(10, 6))

plt.subplot(2, 1, 1)
plt.plot(xref, yref, label='Reference Trajectory',color = "green")
plt.plot(x_interpolated, y_interpolated, label='intended',color = "black")
if(status == "verified"):
    plt.plot(x_interpolated, y_interpolated, label='safe',color = "blue")
else:
    plt.plot(T_safe_x,T_safe_y, label='safe',color = "blue")
    s,v,a,j,u = optimize_longitudinal_trajectory()
    d0 = status[3][0]
    x_long = []
    y_long = []
    for si in s:
        x, y = Yinv(si,d0)
        x_long.append(x)
        y_long.append(y)
    plt.plot(x_long,y_long, label='emergency',color = "red")
    


plt.xlabel('X Position')

plt.ylabel('Y Position')
plt.ylim(0, 20)
# Set y-axis ticks at intervals of 2 units
y_major_locator = MultipleLocator(2)
plt.gca().yaxis.set_major_locator(y_major_locator)
plt.legend()

plt.subplot(2, 1, 2)
#a_traj_degrees = self.orientation()
#plt.plot(self.time[:-1], a_traj_degrees, label='Orientation (a)')
plt.xlabel('Time (s)')
plt.ylabel('Orientation (degrees)')
plt.legend()

plt.tight_layout()
plt.show()
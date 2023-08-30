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
from lat import optimize_lateral_motion
from safe_points import *

T = Ref_path([0, 10], [100, 10], 5)
Tin = intended_traj([(0, 10, 0),(50, 10)])
x_interpolated, y_interpolated = Tin.generate_path()
x0 = Initial_state()
status = x0.initial_state(Tin)
print("")
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
plt.plot(initial_state_long[0] , initial_state_lat[0], marker = 'o' ,color = "black")
plt.plot(20 , 10, marker = 'x' ,color = "red")
plt.plot(x_interpolated, y_interpolated, label='intended',color = "black")
if(status == "verified"):
    plt.plot(x_interpolated, y_interpolated, label='safe',color = "blue")
else:
    ##plan lat from ttr to safe_point_lat
    X0 = x0.initial_state(Tin) 
    i1 = X0[3][: -1]
    s0 = X0[2][0]
    d0 = i1[0]
    sf,df = get_safe_point_lat(s0,d0)
    xf , yf = Yinv(sf,df)
    plt.plot(xf,yf, marker = 'o' ,color = "green")
    f1 = [df,0,0,0]
    x_lat = optimize_lateral_motion(i1,f1)[0]
    d = x_lat[0]
    # s,v,a,j,u = optimize_longitudinal_trajectory()
    x_r = []
    y_r = []
    for di in d:
        x, y = Yinv(s0,di)
        x_r.append(x)
        y_r.append(y)
    plt.plot(x_r,y_r ,label='emergency',color = "red")

    ##plan long from safe_point_lat to safe_point_long
    i2 =  X0[2][: -1]
    s_lon_f, d_lon_f =  get_safe_point_long(s0,df)
    x_lon_f , y_lon_f = Yinv( s_lon_f, d_lon_f)
    plt.plot(x_lon_f , y_lon_f, marker = 'o' ,color = "green")
    f2 =  [s_lon_f,0,0,0]
    s_lon = optimize_longitudinal_trajectory(i2,f2)[0]
    # s,v,a,j,u = optimize_longitudinal_trajectory()
    x_lon_r = []
    y_lon_r = []
    for si in s_lon:
        x, y = Yinv(si,df)
        x_lon_r.append(x)
        y_lon_r.append(y)
    plt.plot(x_lon_r,y_lon_r ,label='emergency',color = "red")

    ##plan lat from safe_point_lon to safe_point_traj
    i3 =  x_lat[:,99]
    s_traj_f, d_traj_f = get_safe_point_traj(s_lon_f, d_lon_f, Tin)
    x_traj_f , y_traj_f = Yinv( s_traj_f, d_traj_f)
    plt.plot(x_traj_f , y_traj_f, marker = 'o' ,color = "green")
    f3 =  [d_traj_f,0,0,0]
    d_traj = optimize_lateral_motion(i3,f3)[0][0]
    # s,v,a,j,u = optimize_longitudinal_trajectory()
    x_traj_r = []
    y_traj_r = []
    for di in d_traj:
        x, y = Yinv(s_lon_f,di)
        x_traj_r.append(x)
        y_traj_r.append(y)
    plt.plot(x_traj_r,y_traj_r ,label='emergency',color = "red")


    ###continue normal intended traj

    


plt.xlabel('X Position')

plt.ylabel('Y Position')
plt.ylim(0, 20)
# Set y-axis ticks at intervals of 2 units
y_major_locator = MultipleLocator(2)
plt.gca().yaxis.set_major_locator(y_major_locator)
plt.legend()

#a_traj_degrees = self.orientation()
#plt.plot(self.time[:-1], a_traj_degrees, label='Orientation (a)')
plt.xlabel('Time (s)')
plt.ylabel('Orientation (degrees)')
plt.legend()

plt.tight_layout()
plt.show()
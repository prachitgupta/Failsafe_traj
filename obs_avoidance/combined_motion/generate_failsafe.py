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
TTR = status[0]

def plot(s,d):
    x_r = []
    y_r = []
    # print(d)
    for si,di in zip(s,d):
        x, y = Yinv(si,di)
        x_r.append(x)
        y_r.append(y)
    plt.plot(x_r,y_r ,label='emergency',color = "red")

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
    ##plan lat from ttr 
    X0 = x0.initial_state(Tin) 
    i_lat_1 = X0[3][: -1]
    i_lon_1 = X0[2][: -1]
    s0 = i_lon_1[0]
    d0 = i_lat_1[0]
    # d0 = i1[0]
    s_safe1,d_safe1 = get_safe_point_lat(s0,d0)
    xf , yf = Yinv( s_safe1,d_safe1)
    plt.plot(xf,yf, marker = 'o' ,color = "green")
    f1 = [d_safe1,0,0,0]

    ##get lon state no final point so pass 0
    print("1st")
    x_long = optimize_longitudinal_trajectory(i_lon_1,0,True)
    ## get lat state
    x_lat =  optimize_lateral_motion(i_lat_1,f1,x_long)[0]
    # x_lat =  optimize_lateral_motion(i_lat_1,0,x_long)[0]
    # generate trajectory 1
    plot(x_long[0], x_lat[0])

    ##GET FINAL S,D FROM TRAJ
    s2f,d2f = x_long[0][99],x_lat[0][99]
    xf , yf = Yinv(s2f,d2f)
    plt.plot(xf,yf, marker = 'o' ,color = "blue")

    ##GET SAFE POINT FROM SET ANALYSIS
    s_safe2 , d_safe2 =  get_safe_point_long( s2f,d2f)

    if (s2f  > s_safe2):
        #plan final
        print("final")
        i_lon_f = np.array([arr[99] for arr in x_long[: -1]])
        i_lat_f = np.array([arr[99] for arr in x_lat])
        s3f, d3f = get_safe_point_traj(s2f,d2f,Tin)
        ##get lon state no final point so pass 0
        ## only continuity  constraint
        i_lon_f[1 :] = [0,0,0]
        x_longf = optimize_longitudinal_trajectory(i_lon_f,0,True)[: -2]
        ## get lat state
        ## only continuity and differentiability constraint
        # x_latf =  optimize_lateral_motion(i_lat_f,0,x_long)[0]
        i_lat_f[2 :] = [0,0]
        ##infeasible due to planned long
        x_latf =  optimize_lateral_motion(i_lat_f,[d3f,0,0,0],x_longf)[0]
        # generate trajectory 2
        plot(x_longf[0], x_latf[0])
      

    else:
   # plan long to safe
        print(f"{ s_safe2} and {s2f}")
        print("middle")
        i_lonm = np.array([arr[99] for arr in x_long[: -1]])
        ## continuity and differentiability constraint extract s, v,a and j = 0
        i_lonm[1 :] = [0,0,0]
        f_lonm = s_safe2
        # x_longm = optimize_longitudinal_trajectory(i_lonm,[f_lonm])[: -2]
        x_longm = optimize_longitudinal_trajectory(i_lonm,[f_lonm],False)
        dm = np.full(100,d2f)
        xf , yf = Yinv(s2f,d2f)
        plt.plot(xf,yf, marker = 'o' ,color = "orange")
        plot(x_longm[0], dm)

    #plan final
        print("last")
        i_lon_f = np.array([arr[99] for arr in x_longm[: -1]])
        i_lat_f =np.array([arr[99] for arr in x_lat])  ## no lateral plannned so same condition befor middle traj
        s3f, d3f = get_safe_point_traj(x_longm[0][99],d2f,Tin)
        ##get lon state no final point so pass 0
        x_longf = optimize_longitudinal_trajectory(i_lon_f,0,False)[: -2]
        ## get lat state
        ## only continuity and differentiability constraints
        i_lat_f[2 :] = [0,0]
        x_latf =  optimize_lateral_motion(i_lat_f,[d3f,0,0,0],x_longf)[0]
        # x_latf =  optimize_lateral_motion(i_lat_f,0,x_long)[0]
        # generate trajectory 1
        plot(x_longf[0], x_latf[0])


    ##integerate with intended

    ## FINAL S MAY NOT LIE ON INTENDED TRAJ , BUT WILL BE VERY CLOSE
    ## SMOOTH TRANSITION TO INTENDED

    


plt.xlabel('X Position')

plt.ylabel('Y Position')
plt.ylim(-10, 30)
# Set y-axis ticks at intervals of 2 units
y_major_locator = MultipleLocator(2)
plt.gca().yaxis.set_major_locator(y_major_locator)
plt.legend()

#a_traj_degrees = self.orientation()
#plt.plot(self.time[:-1], a_traj_degrees, label='Orientation (a)')
plt.xlabel('x')
plt.ylabel('Y')
plt.text(40,6,f"TTR = {TTR}")
plt.legend()

plt.tight_layout()
plt.show()
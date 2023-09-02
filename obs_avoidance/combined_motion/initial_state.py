#!/usr/bin/env python3
from operators import Y,occ
from ref import Ref_path
from intended_arbitary import intended_traj
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
from obstacle import obstacles,ego
from invariably_safe_set import *

## can't find TTR because we generated arbitary intended trajectory using cubic interpolation
## reverse of interpolation is not that easy , anyways we can get x0 but this has to be dealt when we
##consider optimal control real time trajectories
class Initial_state:
    def get_TTR(self, x0):
        x, y = self.x,self.y
        index = np.where(x == x0)[0]
        TTR =  0 + index*(5/100)
        return TTR[0] #approx time when x reaches x0

    def get_v_lat(self,d0):
        d = self.d
        index = np.where(d == d0)[0]
        dt = 0.05
        v_lat0 = (d[index+1] - d[index])/dt
        return v_lat0

    def initial_state(self,Tin):
        self.x, self.y = Tin.generate_path()
        self.d = Tin.generate_curvilinear_path()[:,1]
        state = Tin.state()

        for x0,y0,s0, v0, a0, j0, snap0, d0, theta0, κ0, κ_dot0, κ_ddot0 in (state):
            e = ego(x0,y0,v0)
            Ob = obstacles(20,10)
            if(not is_in_F(e,Ob)):
                print("BOOM!!")
                return "boom"
            else:
                if (not is_in_S(e,Ob)): #i.e safe in infinite time domain
                    TTR = self.get_TTR(x0)
                    # print(f"TTR = {TTR}") 

                    initial_state_long = [s0,v0,a0,j0,snap0]
                    initial_state_lat = [d0 , theta0 , κ0, κ_dot0, κ_ddot0]

                    T_safe = self.x[: np.where(self.x==x0)[0][0]], self.y[: np.where(self.y==y0)[0][0]]
                    #print(I_S.get_v_lat(d0))
                    # print(f"long = {initial_state_long} , lat = {initial_state_lat}")
                    return TTR,T_safe, initial_state_long , initial_state_lat          
        #print("Tin is already safe")
        return("verified")

'''
def visualize():
    plt.figure(figsize=(6, 6))
    plt.scatter(circle_points[:, 0], circle_points[:, 1], color='blue', label='Circle Points')
    plt.scatter(sm, dm, color='red', label='Circle Center')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Circle Occupancy')
    plt.legend()
    plt.grid(True)
    plt.show()


Tin = intended_traj([(1, 11), (4, 12), (30, 11), (50, 10)])
I_S =  Initial_state()
I_S.initial_state(Tin)'
'''
# onboard sensors are used tyo dynamically detect obstacle and set-predictions tools (SPOT) are used to determine there occ
#we are just hard coding dummy obstacles

from ref import Ref_path
from intended_arbitary import intended_traj
from operators import Y,occ
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

T = Ref_path([0, 5], [20, 5], 5)


##obstacle occupancy static assumed
class obstacles:
    def __init__(self,x,y):
        self.obs1_x = x
        self.obs1_y = y
    def limits(self):
        smax_b , smin_b, dmax_b,dmin_b = occ(self.obs1_x,self.obs1_y,1)
        return smax_b , smin_b, dmax_b,dmin_b
    def show_obstacle(self):     
        radius = 1
        x = self.obs1_x 
        y = self.obs1_y 
        num_points = 100
        theta = np.linspace(0, 2*np.pi, num_points)
        
        circle_points_x = x + radius * np.cos(theta)
        circle_points_y = y + radius * np.sin(theta)
        
        circle_points = set(zip(circle_points_x, circle_points_y))
        circle_points = np.array(list(circle_points))
        plt.figure(figsize=(6, 6))
        plt.scatter(circle_points[:, 0], circle_points[:, 1], color='blue', label='Circle Points')
        x_traj,y_traj = T.ref_trajectory()
        plt.plot(x_traj, y_traj, label='Reference Trajectory')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.ylim(0, 10)
        # Set y-axis ticks at intervals of 2 units
        y_major_locator = MultipleLocator(2)
        plt.gca().yaxis.set_major_locator(y_major_locator)
        plt.legend()
        plt.grid(True)
        plt.show()

##occ of ego
class ego:
    def __init__(self,x,y,v_ego):
        self.ego_x = x
        self.ego_y = y
        self.v_ego = v_ego 
    def limits(self):
        smax_b , smin_b, dmax_b,dmin_b = occ(self.ego_x,self.ego_y,1)
        return smax_b , smin_b, dmax_b,dmin_b

# Set an obstacle anywhere for demo

Ob = obstacles(50,10)
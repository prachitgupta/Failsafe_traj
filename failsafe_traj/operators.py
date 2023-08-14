#!/usr/bin/env python3
from ref import Ref_path
import numpy as np
import matplotlib.pyplot as plt

T = Ref_path([0, 10], [100, 10], 5)

def Y(x,y):
    x_ref, y_ref = T.ref_trajectory()
    points = np.column_stack((x_ref, y_ref))
    s_values = np.cumsum(np.sqrt(np.sum(np.diff(points, axis=0) ** 2, axis=1)))
    s_values = np.insert(s_values, 0, 0)  # Add 0 at the beginning for the first point

    # Find the nearest point on the reference trajectory to the given (x, y) coordinates
    distances = np.sqrt((x_ref[:] - x) ** 2 + (y_ref[:] - y) ** 2)
    nearest_index = np.argmin(distances)

    # Calculate the arc length (s) from the beginning of the reference trajectory to the nearest point
    s = s_values[nearest_index]

    # Calculate the orthogonal deviation (d) from the reference trajectory to the given point
    reference_point = points[nearest_index]
    tangent_vector = points[nearest_index + 1] - reference_point
    d = np.dot([x - reference_point[0], y - reference_point[1]], np.array([-tangent_vector[1], tangent_vector[0]])) / np.linalg.norm(tangent_vector)

    return s, d

##assumed straight line
def Yinv(s, d):
    x_ref, y_ref = T.ref_trajectory()
    
    # Calculate the angle of the reference trajectory
    alpha = np.arctan2(y_ref[-1] - y_ref[0], x_ref[-1] - x_ref[0])
    
    # Calculate Cartesian coordinates (x, y) based on curvilinear coordinates (s, d)
    x = x_ref[0] + s * np.cos(alpha) - d * np.sin(alpha)
    y = y_ref[0] + s * np.sin(alpha) + d * np.cos(alpha)
    
    return x, y



#here for sake of simplicity just enlarging occ from a point to circle
def occ(x, y,r):
    radius = r
    num_points = 100
    theta = np.linspace(0, 2*np.pi, num_points)
    
    circle_points_x = x + radius * np.cos(theta)
    circle_points_y = y + radius * np.sin(theta)
    
    #3generate points on circle
    circle_points = set(zip(circle_points_x, circle_points_y))
    ##convert to curvilinear
    curvilinear_coords = []

    for point in circle_points:
        x, y = point
        s, d = Y(x, y)  
        curvilinear_coords.append((s, d))
    
    curvilinear_coords = np.array(curvilinear_coords)
    
    ##find limits on s and d
    s_max,s_min = np.max(curvilinear_coords[:,0]),np.min(curvilinear_coords[:,0])
    d_max,d_min = np.max(curvilinear_coords[:,1]),np.min(curvilinear_coords[:,1])
    return s_max,s_min,d_max,d_min


#sm,smi,dm,dmi = occ(x_center, y_center,1)
#print(list([sm,smi,dm,dm]))
# Plot the circle
'''
circle_points = np.array(list(circle_points))
plt.figure(figsize=(6, 6))
plt.scatter(circle_points[:, 0], circle_points[:, 1], color='blue', label='Circle Points')
plt.scatter(sm, dm, color='red', label='Circle Center')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Circle Occupancy')
plt.legend()
plt.grid(True)
plt.show()
'''



'''
# Plotting the reference trajectory and its orientation
x_coord = 6
y_coord = 6
s_val, d_val = Y(x_coord, y_coord)
print(f"For (x, y) = ({x_coord}, {y_coord}), the curvilinear coordinates are (s, d) = ({s_val}, {d_val})")
'''

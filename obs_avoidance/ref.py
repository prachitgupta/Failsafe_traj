
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

class Ref_path:

    def __init__(self, start, end, T):
        self.x0, self.y0 = start[0], start[1]
        self.x1, self.y1 = end[0], end[1]
        self.T = T

        # Total time duration and number of points in the discrete domain
        num_points = 100
        self.time = np.linspace(0, self.T, num_points)

    # Function to calculate the orientation (slope of tangent) at each point on the reference path
    def orientation_at_point(self, x, y):
        return np.arctan2(y[1:] - y[:-1], x[1:] - x[:-1])

    # Generate the reference trajectory as a function of time (discrete domain)
    def ref_trajectory(self):
        x_traj = self.x0 + (self.x1 - self.x0) * self.time / self.T
        y_traj = self.y0 + (self.y1 - self.y0) * self.time / self.T
        return x_traj, y_traj

    def orientation(self):
        # Calculate the orientation (a) at each point on the reference path
        x_traj, y_traj = self.ref_trajectory()
        a_traj = self.orientation_at_point(x_traj, y_traj)

        # Convert angles to degrees for better visualization
        a_traj_degrees = np.degrees(a_traj)
       # print(a_traj_degrees.shape)
        return a_traj_degrees

    # Plot the reference trajectory and its orientation (slope of tangent) over time
    def plot_ref(self):
        x_traj, y_traj = self.ref_trajectory()

        plt.figure(figsize=(10, 6))

        plt.subplot(2, 1, 1)
        plt.plot(x_traj, y_traj, label='Reference Trajectory')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.ylim(0, 10)
        # Set y-axis ticks at intervals of 2 units
        y_major_locator = MultipleLocator(2)
        plt.gca().yaxis.set_major_locator(y_major_locator)
        plt.legend()

        plt.subplot(2, 1, 2)
        a_traj_degrees = self.orientation()
        plt.plot(self.time[:-1], a_traj_degrees, label='Orientation (a)')
        plt.xlabel('Time (s)')
        plt.ylabel('Orientation (degrees)')
        plt.legend()

        plt.tight_layout()
        plt.show()

# Creating an instance of the Ref_path class
#T = Ref_path([0, 5], [5, 5], 5)
#a = T.orientation()



# Plotting the reference trajectory and its orientation

import cvxpy as cp
import numpy as np

# Define the waypoints for the trajectory
num_waypoints = 4
waypoints = np.array([[0.0, 0.0],
                      [2.0, 3.0],
                      [4.0, 1.0],
                      [5.0, 4.0]])

# Define the bounds for the trajectory (bounding box)
x_min, y_min = -2.0, -2.0
x_max, y_max = 6.0, 6.0

# Define the rectangular obstacles (bounding boxes)
obstacle1 = {'x_min': 1.0, 'x_max': 2.0, 'y_min': 2.0, 'y_max': 3.0}
obstacle2 = {'x_min': 3.0, 'x_max': 4.0, 'y_min': 3.0, 'y_max': 5.0}
obstacles = [obstacle1, obstacle2]

# Main function for trajectory optimization
def optimize_trajectory(waypoints):
    # Number of control inputs (x and y positions)
    num_controls = 2

    # Number of time steps (one less than the number of waypoints)
    num_steps = num_waypoints - 1

    # Initialize the control input variables (2D trajectory)
    trajectory = cp.Variable((num_steps, num_controls))

    # Define the cost function to be minimized (total distance traveled)
    cost = cp.norm(waypoints[0] - trajectory[0])  # Initial distance
    for k in range(num_steps - 1):
        cost += cp.norm(trajectory[k + 1] - trajectory[k])

    # Constraints: Trajectory must stay within bounds and avoid obstacles
    constraints = [
        trajectory >= [x_min, y_min],
        trajectory <= [x_max, y_max],
    ]

    for obstacle in obstacles:
        constraints += [
            trajectory[:, 0] <= obstacle['x_min'],
            trajectory[:, 0] >= obstacle['x_max'],
            trajectory[:, 1] <= obstacle['y_min'],
            trajectory[:, 1] >= obstacle['y_max'],
        ]

    # Create the optimization problem
    problem = cp.Problem(cp.Minimize(cost), constraints)

    # Solve the problem using the chosen solver (e.g., ECOS)
    problem.solve(solver=cp.ECOS)

    return trajectory.value

if __name__ == "__main__":
    optimized_trajectory = optimize_trajectory(waypoints)
    print("Optimized trajectory:")
    print(optimized_trajectory)

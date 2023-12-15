import jax
import jax.numpy as jnp
import numpy as np

from IPython.display import HTML
import matplotlib.animation as anim
import matplotlib.pyplot as plt
import plotly.graph_objects as go

import hj_reachability as hj


class Robot_Human_Simple(hj.ControlAndDisturbanceAffineDynamics):
   def __init__(self,
                 max_robot_acc=3.,
                 max_robot_curvature=0.2,
                 human_max_acc =3.,
                 human_max_turn_rate=0.2,
                 control_mode="max",
                 disturbance_mode="min",
                 control_space=None,
                 disturbance_space=None):
        control_space = hj.sets.Box(jnp.array([-max_robot_acc, - max_robot_curvature]),
                                        jnp.array([max_robot_acc,max_robot_curvature]))
        if disturbance_space is None:
            disturbance_space = hj.sets.Box(jnp.array([-human_max_acc, -human_max_turn_rate]),
                                        jnp.array([human_max_acc, human_max_turn_rate]))
        super().__init__(control_mode, disturbance_mode, control_space, disturbance_space)
   def open_loop_dynamics(self, state, time):
        _, _,yaw_rel,robot_vel,human_vel = state
        return jnp.array([-robot_vel+human_vel*jnp.cos(yaw_rel), human_vel*jnp.sin(yaw_rel),0,0,0])

   def control_jacobian(self, state, time):
        #print(state)
        x_rel, y_rel, _ ,robot_vel, _ = state
        return jnp.array([
            [0., robot_vel*y_rel],
            [0.,-robot_vel*x_rel],
            [0., -robot_vel],
            [1, 0],
            [0., 0.],
        ])


   def disturbance_jacobian(self, state, time):
        return jnp.array([
            [0., 0.],
            [0., 0.],
            [0., 1],
            [0., 0.],
            [1, 0.],
        ])

dynamics = Robot_Human_Simple()
## grid states bounds and no. of discretization of each state
grid = hj.Grid.from_lattice_parameters_and_boundary_conditions(hj.sets.Box(lo=np.array([-15., -5., -(np.pi)/2, 1,1]),
                                                                           hi=np.array([15., 5.,  (np.pi)/2,12,12])),
                                                                          (13,13,9,9,9),
                                                               periodic_dims=2)
##circle of 4m is selected as collision radius i.e target set
initialValues = jnp.linalg.norm(grid.states[..., :2], axis=-1) - 4
solver_settings = hj.SolverSettings.with_accuracy("very_high",
                                                  hamiltonian_postprocessor=hj.solver.backwards_reachable_tube)

time = 0.
target_time = -2.0
targetValues = hj.step(solver_settings, dynamics, grid, time, initialValues, target_time)

#2d (x-y) slice of value function at given yaw , vr and vh
[yaw,vr,vh] = grid.coordinate_vectors[2:5]
##computr optimal control 
##grad values grid, targetValues)
grad_values = grid.grad_values(targetValues)
states = grid.states
epis  = 0.003
## find state at which V <= epis
i = jnp.where(targetValues < epis)
num_indices = len(i[0])
#any arbitary grid point, technically should be that state vector at which V < epis for the first time
state = states[ -1 ,-1,-1,-1,-1,:]  
##OPTIMAL CONTROL ANDD DIST
Uopt, Dopt = dynamics.optimal_control_and_disturbance(state,time,grad_values)
yaw_slice,vr_slice,vh_slice = np.where(yaw ==-0.17453295),np.where(vr ==   7.875 ),np.where(vh ==  7.875)
print(f"BRS at yaw = {yaw[yaw_slice]} , speed human = {vh[vh_slice]} , speed robot = {vr[vr_slice]} ")
plt.jet()
plt.figure(figsize=(13, 8))
plt.contourf(grid.coordinate_vectors[0], grid.coordinate_vectors[1], targetValues[:, :,4,5,5].T)
plt.colorbar()
plt.contour(grid.coordinate_vectors[0],
            grid.coordinate_vectors[1],
            targetValues[:, :,4,5,5].T,
            levels=0,
            colors="black",
            linewidths=3)
print(f"optimal control = {Uopt}")
# Failsafe_traj

- Following Repo is the implementation of the given research paper C. Pek and M. Althoff, "Fail-Safe Motion Planning for Online Verification of Autonomous Vehicles Using Convex Optimization," in IEEE Transactions on Robotics, vol. 37, no. 3, pp. 798-814, June 2021, doi: 10.1109/TRO.2020.3036624. in python using CVXPY packages for convex optimization

- Various operators, occupancy, and conditions for safety in infinite time horizons as described in the paper have been implemented as separate classes giving the user access to modify accordingly

- Note: This code snippet is for demonstration purposes only with generate.py demonstrating collision avoidance against static obstacles via emergency braking
  
- Vehicle models for both lateral and longitudinal motions are modeled as LTI systems under given assumptions and under the approximation of a safe set enabling us to generate only linear constraints. Thus optimization problem is modelled as convex optimization enabling us to use open-source modelling language like CVXPY and guaranteeing convergence for the infinite time horizon

- Here reference and intended trajectories are just examples random trajectories, users can their own trajectories for verification by the online verification technique
  
The same goes for obstacle and ego objects and constraints on optimization problems

- Basic template for the algorithm ensuring only provable safe trajectories get executed for any planner giving real-time trajectories has been provided in integrate.py. It can be used as a reference for integrating the Failsafe trajectories in various projects

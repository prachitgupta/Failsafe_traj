## overtaking leading vehicle moving with constant velocity
import sys
sys.path.append("/home/prachit/Desktop/failsafe_traj/Failsafe_traj/my_scripts")
from pid_control import PIDcontrol 
import carla
import numpy as np
import cv2 as cv
import queue
import math

# spawn our vehicle put in autopilot
def ego_control(vehicle,vdes):
    vehicle.set_autopilot(True)
    #vehicle.set_velocity(carla.Vector3D(x=vdes, y=0, z=0))

def obs_control(vehicle, vdes):
    vehicle.set_autopilot(True)
    #vehicle.set_velocity(carla.Vector3D(x=vdes, y=0, z=0)) 


##move leading vehicle with constant velocity



##extract state in time domain and pass to mpc

if __name__ == "__main__":
    actor = []
    try :
        ##set up environment
        client = carla.Client('localhost',2000)
        client.set_timeout(20.0)
        world = client.load_world('Town04')
        map = world.get_map()
        blueprint = world.get_blueprint_library()
        ego_bp =   blueprint.filter('cybertruck')
        obs_bp = blueprint.filter('cybertruck')
        spawn_points = map.get_spawn_points()
        ego_vehicle = world.spawn_actor(ego_bp[0],spawn_points[30])     #SPWAN POINTS TO BE CONSIDERED  60,61,62,63
        actor.append(ego_vehicle)
        spectator = world.get_spectator()
        transform = ego_vehicle.get_transform()
       # print(carla.Transform(transform.location + carla.Location(x=5,y=0,z=3)))
        obs_transform = carla.Transform(transform.location + carla.Location(x=60,y=0,z=3) , transform.rotation)
        obs_vehicle = world.spawn_actor(obs_bp[0],obs_transform)
        spectator.set_transform(carla.Transform(transform.location + carla.Location(x=2,y=0,z=3),
        carla.Rotation(pitch= 0 ,yaw = 0)))   ##set spectator view
        # wps = map.get_waypoint(ego_vehicle.get_location())
        # print(wps)

    
        ego_control(ego_vehicle,vdes = 5)
        obs_control(obs_vehicle,vdes =  70*5/18)


        
    finally :
        client.apply_batch([carla.command.DestroyActor(i) for i in actor])
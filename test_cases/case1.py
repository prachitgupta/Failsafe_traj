## overtaking leading vehicle moving with constant velocity
from pid_control import PIDcontrol
# from controller import  VehiclePIDController, PIDLateralController, PIDLongitudinalController
import carla
import numpy as np
import cv2 as cv
import queue
import math
# from Util_functions import get_speed

# spawn our vehicle put in autopilot
def ego_control(vehicle,vdes):
    vehicle.set_autopilot(True)
    #vehicle.set_velocity(carla.Vector3D(x=vdes, y=0, z=0))

def obs_control(vehicle, vdes, wayp):

    #simple pid control
    control_vehicle = PIDcontrol(vehicle, arg_lat = {'Kp':1.95,'Kd':.05,'Ki':0.2}, arg_long = {'Kp':1,'Kd':0.05,'Ki':0}, max_thr=  0.75, max_steer = 0.3, max_break = 0.8 )
    control_signal = control_vehicle.run_step(vdes,wayp)
    obs_vehicle.apply_control(control_signal)




##extract state in time domain and pass to mpc
def get_state(vehicle):
    transform = vehicle.get_transform()

    # Get the vehicle's velocity
    velocity = vehicle.get_velocity()

    # Extract the longitude, latitude, and velocity components
    lon = transform.location.y
    lat = transform.location.x
    speed = velocity.length()
    return [lon, lat, speed]

if __name__ == "__main__":
    actor = []
    try :
        ##set up environment
        client = carla.Client('localhost',2000)
        client.set_timeout(20.0)
        world = client.load_world('Town04')
        map = world.get_map()
        blueprint = world.get_blueprint_library()
        ego_bp =  blueprint.find('vehicle.lincoln.mkz_2020') 
        obs_bp = blueprint.find("vehicle.toyota.prius")
        spawn_points = map.get_spawn_points()
        ego_vehicle = world.spawn_actor(ego_bp,spawn_points[30])     #SPWAN POINTS TO BE CONSIDERED  60,61,62,63
        actor.append(ego_vehicle)
        spectator = world.get_spectator()
        transform = ego_vehicle.get_transform()
       # print(carla.Transform(transform.location + carla.Location(x=5,y=0,z=3)))
        obs_transform = carla.Transform(transform.location + carla.Location(x=0,y=10,z=0) , transform.rotation)
       # print(f"{obs_transform} from {transform}")
        obs_vehicle = world.spawn_actor(obs_bp,obs_transform)
        if ego_vehicle is not None and obs_vehicle is not None:
            print(f"Spawned two  vehicles")
        else:
            print("Failed to spawn vehicles.")

       ##spectator
        camera_loc = carla.Location(x=436.332245, y=-58.211384, z=26.967960)
        camera_rot = carla.Rotation(pitch=-49.312428, yaw=179.294144, roll=0.000040)
        spectator.set_transform(carla.Transform(camera_loc,camera_rot))
        
        while True:
            wps = map.get_waypoint(obs_vehicle.get_location())
            # select 1 waypoint randomly from list of new wp in radius 0.3 from wps
            wp = np.random.choice(wps.next(2))
            print(wp)
            #wp = np.random.choice(wps, 1)
            ##pass step input desired velocity and target wp 
            obs_control(obs_vehicle,50*5/18,wp)
            print(get_state(obs_vehicle))
            ego_control(ego_vehicle,vdes = 5)


        
    finally :
        client.apply_batch([carla.command.DestroyActor(i) for i in actor])
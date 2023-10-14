## overtaking leading vehicle moving with constant velocity
import sys
sys.path.append("/home/prachit/Desktop/failsafe_traj/Failsafe_traj/my_scripts")
from pid_control import PIDcontrol,LongControl, LatControl
import carla
import numpy as np
import cv2 as cv
import queue
import math

# spawn our vehicle put in autopilot
def ego_control(vehicle,vdes):
    vehicle.set_autopilot(True)
    #vehicle.set_velocity(carla.Vector3D(x=vdes, y=0, z=0))


def obs_control(vehicle, vdes, wayp):

    #simple pid control
    control_vehicle = PIDcontrol(vehicle, arg_lat = {'Kp':1,'Kd':0,'Ki':0}, arg_long = {'Kp':1,'Kd':0,'Ki':0}, max_thr=  0.75, max_steer = 0.3, max_break = 0.8 )
    control_signal = control_vehicle.run_step(vdes,wayp)
    vehicle.apply_control(control_signal)


    # ##just move with constant velocity
    # target_velocity = vdes  # Change this to your desired velocity
    # control = carla.VehicleControl()
    # control.throttle = 1.0  # Full throttle
    # control.steer = 0.0  # Straight steering
    # control.brake = 0.0  # No brakes
    # control.hand_brake = False
    # # Set the desired velocity
    # control.target_velocity = target_velocity
    # control.use_constant_velocity = True
    # # Apply the control commands
    # vehicle.apply_control(control)



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
        leading_bp = blueprint.find("vehicle.toyota.prius")
        opposite_bp = blueprint.find('vehicle.seat.leon') 
        spawn_points = map.get_spawn_points()
        ego_vehicle = world.spawn_actor(ego_bp,spawn_points[30])     #SPWAN POINTS TO BE CONSIDERED  60,61,62,63
        actor.append(ego_vehicle)
        spectator = world.get_spectator()
        transform = ego_vehicle.get_transform()
       # print(carla.Transform(transform.location + carla.Location(x=5,y=0,z=3)))
        leading_transform = carla.Transform(transform.location + carla.Location(x=0,y=10,z=0) , transform.rotation)
        opposite_transform = carla.Transform(transform.location + carla.Location(x=5,y=50,z=0) , carla.Rotation(yaw=270.0))
       # print(f"{obs_transform} from {transform}")
        leading_vehicle = world.spawn_actor(leading_bp,leading_transform)
        actor.append(leading_vehicle)
        opposite_vehicle = world.spawn_actor(opposite_bp,opposite_transform)
        actor.append(opposite_vehicle)
        if ego_vehicle is not None and leading_vehicle is not None  and opposite_vehicle is not None:
            print(f"Spawned all  vehicles")
        else:
            print("Failed to spawn vehicles.")

        ##spectator
        camera_loc = carla.Location(x=436.332245, y=-58.211384, z=26.967960)
        camera_rot = carla.Rotation(pitch=-49.312428, yaw=179.294144, roll=0.000040)
        spectator.set_transform(carla.Transform(camera_loc,camera_rot))
        
        while True:
            wps1 = map.get_waypoint(leading_vehicle.get_location())
            wp1 = np.random.choice(wps1.next(0.3))
            wps2 = map.get_waypoint(leading_vehicle.get_location())
            wp2 = np.random.choice(wps2.next(0.3))

            ##move leading with constant velocity 50km/hr
            obs_control(leading_vehicle,50*(5/18),wp1)
            state_leading = get_state(leading_vehicle)

            ##move opposite vehicle with 70 in oppositr direction
            obs_control(opposite_vehicle,70*(5/18),wp2)
            state_opposite = get_state(opposite_vehicle)

            ##ego with mpc
            ego_control(ego_vehicle,vdes = 5)
            #print(spectator.get_transform())


        
    finally :
        client.apply_batch([carla.command.DestroyActor(i) for i in actor])
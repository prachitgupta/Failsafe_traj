import carla
import numpy as np
import cv2 as cv
import queue
import math

def get_speed(vehicle):
        vel = vehicle.get_velocity()
        return math.sqrt(vel.x**2 + vel.y**2 + vel.z**2 )

class PIDcontrol:
    def __init__(self, ego , arg_lat, arg_long, max_thr, max_steer, max_break):
        self.ego = ego
        self.max_thr = max_thr
        self.max_steer = max_steer
        self.max_brak = max_break

        self.world = ego.get_world()
        self.long_control = LongControl(self.ego, **arg_long)
        self.lat_control = LatControl(self.ego, **arg_lat)

    def run_step(self, v_target, waypoint):
        ##long controller returns acc
        acc = self.long_control.run_step(v_target)
        ##lat control gives desired steering
        current_steering = self.lat_control.run_step(waypoint)
        ## create carla control object to send control commands to vehicle
        control = carla.VehicleControl()
        ## control input is acc and steering
        return control

class LongControl:
    def __init__(self,ego, Kp , Kd, Ki, dt = 0.03):
        self.ego = ego
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki

        ##define error buffer deque quicker append and pop operations than list
        self.errorBuffer = queue.deque(maxLen = 10)

    
    def run_step(self, v_target):
        ##long controller returns acc
        v_current = get_speed(self.ego)
        ##apply pid on speed
        return self.pid_controller(self, v_target, v_current)
    
    def pid_controller(self,target, current):
        error = target - current

        self.errorBuffer.append(error)

        if len(self.errorBuffer) >= 2:
            de = (self.errorBuffer[-1] - self.errorBuffer[-2])/self.dt
            cum_error = sum(self.errorBuffer)*self.dt

        else :
            de = 0
            cum_error = 0 
        ##pid control input between -1 and 1 values less than -1 set to -1 
        return ( np.clip(self.Kp*error + self.Kd*de + self.Ki*cum_error, -1 , 1))
class LatControl:
    def __init__(self,ego, Kp , Kd, Ki, dt = 0.03):
        self.ego = ego
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki

        ##define error buffer deque quicker append and pop operations than list
        self.errorBuffer = queue.deque(maxLen = 10)
    def run_step(self,v_target, waypoint):
        ##long controller returns acc
        acc = self.long_control.run_step(v_target)
        ##lat control gives desired steering
        current_steering = self.lat_controller.run_step(waypoint)


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
        spawn_points = map.get_spawn_points()
        ego_vehicle = world.spawn_actor(ego_bp[0],spawn_points[30])     #SPWAN POINTS TO BE CONSIDERED  60,61,62,63
        actor.append(ego_vehicle)
        spectator = world.get_spectator()
        transform = ego_vehicle.get_transform()
        print(carla.Transform(transform.location + carla.Location(x=5,y=0,z=3)))
        spectator.set_transform(carla.Transform(transform.location + carla.Location(x=5,y=0,z=3),
        carla.Rotation(pitch= 0 ,yaw = 0)))   ##set spectator view
        # wps = map.get_waypoint(ego_vehicle.get_location())
        # print(wps)

        control_vehicle = PIDcontrol(ego_vehicle, args_lat = {'Kp':1,'Kd':0,'Ki':0}, args_long = {'Kp':1,'Kd':0,'Ki':0}, max_thr=  0.75, max_steer = 0.3, max_break = 0.8 )
        while True:
            #getting the closest waypoint to a specific location 
            wps = map.get_waypoint(ego_vehicle.get_location())
            #wps = np.random.choice(wps(0.3))
            wp = np.random.choice(wps, 1)
            ##pass step input desired velocity and target wp 
            control_signal = control_signal.run_step(5,wp)



    finally :
        client.apply_batch([carla.command.DestroyActor(i) for i in actor])


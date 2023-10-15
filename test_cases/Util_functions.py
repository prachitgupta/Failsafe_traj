#!usr/bin/env pyhton3


# Author: Rugved Katole
# Affliation: Indian Institute of Bombay
# Date: 8 June 2022


import math
import numpy as np
import scipy.special
import carla
def get_speed(vehicle):
    """compute speed of vehicle in km/h
    function calculates magnitude of velocity vector"""

    vel = vehicle.get_velocity()

    return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

def distance (pose1,pose2, option="pose"):
    """
    param pose format: [x,y]
    option: 'carla.Transform' , 'carla.Location', pose[x,y]"""
    if option == "transform":
        pose1 = [pose1.location.x, pose1.location.y]
        pose2 = [pose2.location.x, pose2.location.y]
    return math.sqrt((pose1[0]-pose2[0])**2 + (pose1[1]-pose2[1])**2)

def get_x_y(location):
    """return x,y from carla.Location class"""
    if isinstance(location, carla.Location):
        x = location.x
        y = location.y
    
    if isinstance(location,carla.Transform):
        x = location.location.x
        y = location.location.y

    return [x,y]

def Bezier_curve(u,control_points):
    """
    Returns a point on the bezier curve
    :param t: float parameter in [0,1]
    :param control_points: numpy array of control points
    :returnL Co-ordinates of points
    """
    control_points = np.array(control_points)
    
    n = len(control_points) - 1
    return np.array([np.sum([berstein_poly(n,i,u)*control_points[i,0] for i in range(n + 1)],axis = 0),np.sum([berstein_poly(n,i,u)*control_points[i,1] for i in range(n + 1)],axis = 0)])

def berstein_poly(n,i,u):
    """
    ith term of a Berstein polynomial of n degree
    :param n: degree of the polynomial
    :param i: ith term
    :param t: parametic constant t

    """
    return scipy.special.comb(n,i)* u**i * (1-u)**(n-i)

def bezier_derivatives_control_points(control_points, n_derivatives):
    """
        Compute control points of the successive derivatives of a given bezier curve.
    A derivative of a bezier curve is a bezier curve.
    See https://pomax.github.io/bezierinfo/#derivatives
    for detailed explanations
    :param control_points: (numpy array)
    :param n_derivatives: (int)
    e.g., n_derivatives=2 -> compute control points for first and second derivatives
    :return: ([numpy array])
    """
    control_points = np.array(control_points)
    w = {0: control_points}
    for i in range(n_derivatives):
        n = len(w[i])
        w[i + 1] = np.array([(n - 1) * (w[i][j + 1] - w[i][j]) for j in range(n - 1)])
    return w


if __name__ == "__main__":

    client = carla.Client("localhost",2000)
    client.set_timeout(20.0)
    world = client.get_world()
    print(world)
    world = client.load_world('Town06')
    vehicle_blueprints = world.get_blueprint_library().filter("cybertruck")
    spawn_points = world.get_map().get_spawn_points()
    ego_vehicle = world.spawn_actor(vehicle_blueprints[0],spawn_points[25])     #SPWAN POINTS TO BE CONSIDERED  60,61,62,63

    spectator = world.get_spectator()
    transform = ego_vehicle.get_transform()
    spectator.set_transform(carla.Transform(transform.location + carla.Location(x=5,y=0,z=3),
    carla.Rotation(pitch=-20,yaw = 180)))

    ego_wpt = world.get_map().get_waypoint(ego_vehicle.get_location())
    print(ego_wpt)
    print(get_x_y(ego_wpt.transform.location))
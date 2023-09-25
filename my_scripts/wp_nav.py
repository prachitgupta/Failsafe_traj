import carla
# import some code coming with the sim
import os
import sys
# Get the user's home directory
home_dir = os.path.expanduser("~")

# Construct the full path to the carla directory
carla_dir = os.path.join(home_dir, 'Desktop/failsafe_traj/CARLA_0.9.14/PythonAPI/carla')

# Add the directory to sys.path
sys.path.append(carla_dir)


client = carla.Client('localhost', 2000)

world = client.get_world()
spawn_points = world.get_map().get_spawn_points()



# vehicle_all = world.get_blueprint_library()
# print(vehicle_all)



vehicle_bp = world.get_blueprint_library().filter('*volkswagen*')
start_point = spawn_points[0]
vehicle = world.try_spawn_actor(vehicle_bp[0], start_point)

# making a route
#from one position to another
#pos 1: Transform(Location(x=50.477512, y=141.135620, z=0.001844), Rotation(pitch=0.000007, yaw=0.318098, roll=0.000000))
#pos 2: Transform(Location(x=-64.644844, y=24.471010, z=0.600000), Rotation(pitch=0.000000, yaw=0.159198, roll=0.000000))


# using the code to plan the route and then draw it in the simulator
#town10hd_map = world.get_map()

# # utility script of destruction

# for actor in world.get_actors().filter('*vehicle*'):
#     actor.destroy()
# for sensor in world.get_actors().filter('*sensor*'):
#     sensor.destroy()

# now we define 2 cars
truck_bp = world.get_blueprint_library().filter('*firetruck*')
mini_bp = world.get_blueprint_library().filter('*cooper_s*')

#start first car in alredy defined start point
truck = world.try_spawn_actor(truck_bp[0], start_point)

# tweak spectator position to watch the show

spectator = world.get_spectator()
spawn_points = world.get_map().get_spawn_points()
start_point = spawn_points[0]

spectator_pos = carla.Transform(start_point.location + carla.Location(x=20,y=10,z=4),
                                carla.Rotation(yaw = start_point.rotation.yaw -155))

spectator.set_transform(spectator_pos)


# drop the Mini the sky - watch what happens after

#spawn it first somewhere else
mini = world.try_spawn_actor(mini_bp[0], spawn_points[10])

## actual wp nav move from start to goal with autopilot

mini_pos = carla.Transform(start_point.location + carla.Location(x=-4,z=10),
                            carla.Rotation(yaw = start_point.rotation.yaw - 0))
mini.set_transform(mini_pos)






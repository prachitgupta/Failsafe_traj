import carla

client = carla.Client('localhost', 2000)

world = client.get_world()
spawn_points = world.get_map().get_spawn_points()



# vehicle_all = world.get_blueprint_library()
# print(vehicle_all)



vehicle_bp = world.get_blueprint_library().filter('*volkswagen*')
start_point = spawn_points[0]
vehicle = world.try_spawn_actor(vehicle_bp[0], start_point)

vehicle_pos = vehicle.get_transform()
print(vehicle_pos)



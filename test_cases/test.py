import carla

def spawn_vehicle(world, blueprint_name, distance):
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.find(blueprint_name)

    # Define spawn points for the vehicles
    spawn_point1 = carla.Transform(carla.Location(x=0, y=0, z=1), carla.Rotation(yaw=0))
    spawn_point2 = carla.Transform(carla.Location(x=distance + 100, y=0, z=1), carla.Rotation(yaw=0))

    # Spawn the vehicles
    vehicle1 = world.try_spawn_actor(vehicle_bp, spawn_point1)
    vehicle2 = world.try_spawn_actor(vehicle_bp, spawn_point2)

    if vehicle1 is not None and vehicle2 is not None:
        print(f"Spawned two {blueprint_name} vehicles at a distance of {distance} meters.")
    else:
        print("Failed to spawn vehicles.")

if __name__ == "__main__":

    # Connect to the CARLA simulator
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()

    # Define the blueprint name for the vehicles (you can change this)
    blueprint_name = "vehicle.audi.etron"

    # Specify the distance between the two vehicles (20 meters in this case)
    distance_between_vehicles = 20.0

    # Call the spawn_vehicle function to spawn two vehicles
    spawn_vehicle(world, blueprint_name, distance_between_vehicles)
    
    # finally:
    #     # Cleanup when done
    #     client.disconnect()


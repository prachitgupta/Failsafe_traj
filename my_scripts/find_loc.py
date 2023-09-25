import carla

# Connect to the CARLA simulator
client = carla.Client('localhost', 2000)
client.set_timeout(2.0)  # Set a timeout for the connection

# Get the world object
world = client.get_world()

# Define the coordinates of the point you want to find
point_location = carla.Location(x=100.0, y=200.0, z=0.0)  # Adjust the coordinates as needed

# Use a collision query to find the location of the point in the world
hit_result = world.get_hit_location(point_location)

# Print the coordinates of the point in the world
print(f"Coordinates of the point in the world: {hit_result.location}")

import carla

client = carla.Client("localhost", 2000)
client.set_timeout(10.0)

world = client.get_world()
carla_map = world.get_map()

print("Current map:", carla_map.name)

spawn_points = carla_map.get_spawn_points()
"""
import carla

client = carla.Client("localhost", 2000)
client.set_timeout(10.0)

world = client.get_world()
carla_map = world.get_map()

print("Current map:", carla_map.name)

spawn_points = carla_map.get_spawn_points()

for i, point in enumerate(spawn_points):
    print(
        f"{i}: "
        f"x={point.location.x:.2f}, "
        f"y={point.location.y:.2f}, "
        f"z={point.location.z:.2f}, "
        f"yaw={point.rotation.yaw:.2f}"
    ) """
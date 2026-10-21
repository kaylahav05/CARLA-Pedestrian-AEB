import carla
import time
from simulation.vehicle import Vehicle

def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    world = client.get_world()
     #spectator = world.get_spectator()

    """ attaching spectator to vehicle object (after vehicle spawns)
    vehicle_transform = vehicle.actor.get_transform()
    spectator.set_transform(
            carla.Transform(
                vehicle_transform.transform(
                    carla.Location(
                        x=-8, z=4) ),
                    carla.Rotation(
                        pitch=-15,
                        yaw=vehicle_transform.rotation.yaw
                        )   
                    )
                )
    """

    


if __name__ == "__main__":
    main()
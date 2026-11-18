
import carla
import time

from simulation.scenario import Scenario
from tests.scenarioconfig import SCENARIOS
from perception.camera import Camera


def main():
    # Connect to CARLA
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    original_settings = world.get_settings()
    scenario = None
    camera = None


    print("RGB camera attached successfully")

    try:
        # Configure synchronous simulation
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        # Create and set up scenario
        config = SCENARIOS["basic_crossing"]
        scenario = Scenario(world, config)

        if not scenario.setup():
            print("Scenario setup failed")
            return

        camera = Camera(world, scenario.vehicle.actor)
        camera.attach_camera()
        print("RGB camera attached successfully")

        # Update actor positions after spawning
        world.tick()

        # Select a destination along the vehicle's lane
        carla_map = world.get_map()
        vehicle_location = scenario.vehicle.get_location()

        waypoint = carla_map.get_waypoint(
            vehicle_location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )

        if waypoint is None:
            raise RuntimeError("No driving waypoint found")

        next_waypoints = waypoint.next(60.0)

        if not next_waypoints:
            raise RuntimeError("No waypoint found 60 meters ahead")

        destination = next_waypoints[0].transform.location
        scenario.vehicle.set_destination(destination)


        # Start scenario
        if not scenario.start():
            print("Scenario start failed")
            return

        print("Scenario started")

        spectator = world.get_spectator()

        # Main simulation loop
        while not scenario.is_complete():
            scenario.update()
            world.tick()

            # Display RGB camera feed
            camera.display()

            # Follow vehicle with spectator camera
            vehicle_transform = scenario.vehicle.actor.get_transform()

            spectator.set_transform(
                carla.Transform(
                    vehicle_transform.transform(
                        carla.Location(x=-8.0, z=5.0)
                    ),
                    carla.Rotation(
                        pitch=-20.0,
                        yaw=vehicle_transform.rotation.yaw
                    )
                )
            )

            # Slow simulation for visual observation
            time.sleep(0.05)

        print("Scenario completed")

    finally:
        if camera is not None:
            camera.destroy()

        if scenario is not None:
            scenario.cleanup()

        world.apply_settings(original_settings)


if __name__ == "__main__":
    main()

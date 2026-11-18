
import carla
import time

from simulation.scenario import Scenario
from tests.scenarioconfig import SCENARIOS
from perception.camera import Camera
from perception.detector import Detector


def main():

    # Connect to CARLA
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    world = client.get_world()

    print("Connected to CARLA")
    print("Current map:", world.get_map().name)

    # Save original world settings
    original_settings = world.get_settings()

    scenario = None
    camera = None

    try:
        # Enable synchronous simulation
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        # Select scenario configuration
        config = SCENARIOS["basic_crossing"]

        # Create and set up scenario actors
        scenario = Scenario(world, config)

        if not scenario.setup():
            print("Scenario setup failed")
            return

        # Advance simulation to initialize actor positions
        world.tick()

        # Get vehicle's actual location
        vehicle_location = scenario.vehicle.get_location()


        # Initialize BasicAgent after first tick
        if not scenario.vehicle.setup_agent():
            print("BasicAgent setup failed")
            return

        # Attach RGB camera to vehicle
        camera = Camera(world, scenario.vehicle.actor)
        camera.attach_camera()
        print("RGB camera attached successfully")

        # Initialize pedestrian detector
        detector = Detector(
            world,
            scenario.vehicle.actor,
            scenario.pedestrian.actor
)


        # Start scenario
        if not scenario.start():
            print("Scenario start failed")
            return

        print("Scenario started")

        # Set up spectator camera
        spectator = world.get_spectator()

        # Main simulation loop
        while not scenario.is_complete():

            # Update vehicle and pedestrian movement
            scenario.update()

            # Advance simulation
            world.tick()

            # Display camera feed
            camera.display()

            # Run YOLO pedestrian detection
            if camera.frame is not None:
                detections = detector.detect_pedestrians(camera.frame)
                print("Pedestrians detected:", detections)

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

            # Slow simulation for visual testing
            time.sleep(0.05)

        print("Scenario completed")

    finally:

        if camera is not None:
            camera.destroy()

        # Destroy vehicle and pedestrian
        if scenario is not None:
            scenario.cleanup()

        # Restore original CARLA settings
        world.apply_settings(original_settings)

        print("Simulation cleaned up")


if __name__ == "__main__":
    main()

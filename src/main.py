import carla
import time

from simulation.scenario import Scenario
from perception.camera import Camera
from perception.detector import Detector
from tests.config_gen import crossing_config

AEB_ENABLED = True      # set False to check the scenario geometry (should collide)
REPEATS = 1             # runs per scenario

# (name, kwargs for crossing_config)
TEST_CASES = [
    ("30 kmh from left",     dict(v_kmh=30, side="left")),
    ("30 kmh from right",    dict(v_kmh=30, side="right")),
    ("20 kmh",               dict(v_kmh=20)),
    ("40 kmh",               dict(v_kmh=40)),
    ("50 kmh",               dict(v_kmh=50)),
    ("slow walker 1.4 m/s",  dict(v_kmh=30, ped_speed=1.4)),
    ("clears early (+1.5s)", dict(v_kmh=30, lead=1.5)),
    ("arrives late (-1.5s)", dict(v_kmh=30, lead=-1.5)),
    ("standing in lane",     dict(v_kmh=30, ped_speed=0, d_lat=0)),
    ("standing on sidewalk", dict(v_kmh=30, ped_speed=0, d_lat=5.4)),
]


def run_scenario(world, config, aeb_enabled=True):
    """Your original loop, run once for one config. Returns a result dict."""
    result = {"collided": False, "impact_speed": None,
              "min_dist": float("inf"), "error": None}

    scenario = None
    camera = None
    sensor = None
    hits = []
    detections = {}
    path_detections = []
    distance = 0
    ped_in_path = False

    try:
        # Create and set up scenario actors
        scenario = Scenario(world, config)

        if not scenario.setup():
            result["error"] = "scenario setup failed"
            return result

        # Advance simulation to initialize actor positions
        world.tick()

        # Initialize BasicAgent after first tick
        if not scenario.vehicle.setup_agent():
            result["error"] = "BasicAgent setup failed"
            return result

        # Start at target speed so the generator's timing math holds
        veh = scenario.vehicle.actor
        ped = scenario.pedestrian.actor
        v = config["vehicle_target_speed"] / 3.6
        fwd = veh.get_transform().get_forward_vector()
        veh.set_target_velocity(carla.Vector3D(v * fwd.x, v * fwd.y, 0))

        # Collision sensor (pedestrian hits only)
        bp = world.get_blueprint_library().find("sensor.other.collision")
        sensor = world.spawn_actor(bp, carla.Transform(), attach_to=veh)
        sensor.listen(lambda e: hits.append(e.other_actor.type_id)
                      if "walker" in e.other_actor.type_id else None)

        # Attach RGB camera to vehicle
        camera = Camera(world, veh)
        camera.attach_camera()

        # Initialize pedestrian detector
        detector = Detector(world, ped, veh)

        # Start scenario
        if not scenario.start():
            result["error"] = "scenario start failed"
            return result

        spectator = world.get_spectator()

        # Main simulation loop
        while not scenario.is_complete():
            detections = {}
            path_detections = []
            threat_detections = []
            distance = 0
            ped_in_path = False

            # Run YOLO pedestrian detection
            if camera.frame is not None:
                detections = detector.detect_pedestrians(camera.frame)

                if detections:
                    distance = detector.get_pedestrian_distance()
                    for detection in detections:
                        if detector.is_in_path(detection):
                            path_detections.append(detection)

                        if detector.is_near_path(detection):
                            threat_detections.append(detection)

                ped_in_path = len(path_detections) > 0

            # Update vehicle and pedestrian movement
            scenario.update(threat_detections, distance, ped_in_path, aeb_enabled)
            speed = scenario.vehicle.get_speed()

            # Advance simulation
            world.tick()

            # Display camera feed
            camera.display(detector.path_polygon)

            # Follow vehicle with spectator camera
            vehicle_transform = veh.get_transform()
            spectator.set_transform(
                carla.Transform(
                    vehicle_transform.transform(carla.Location(x=-8.0, z=5.0)),
                    carla.Rotation(pitch=-20.0, yaw=vehicle_transform.rotation.yaw)
                )
            )

            # Metrics
            result["min_dist"] = min(result["min_dist"],
                                     veh.get_location().distance(ped.get_location()))
            if hits:
                result["collided"] = True
                result["impact_speed"] = speed
                break

            # Slow simulation for visual testing
            time.sleep(0.05)

    finally:
        if sensor is not None:
            sensor.stop()
            sensor.destroy()
        if camera is not None:
            camera.destroy()
        if scenario is not None:
            scenario.cleanup()
        world.tick()    # let CARLA remove the destroyed actors before the next spawn

    return result


def main():

    # Connect to CARLA
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    world = client.get_world()

    print("Connected to CARLA")
    print("Current map:", world.get_map().name)

    # Save original world settings
    original_settings = world.get_settings()

    summary = []

    try:
        # Enable synchronous simulation
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        for name, kwargs in TEST_CASES:
            for run in range(REPEATS):
                print(f"\n=== {name} (run {run + 1}/{REPEATS}) ===")
                config = crossing_config(**kwargs)
                result = run_scenario(world, config, AEB_ENABLED)
                summary.append((name, run + 1, result))

    finally:
        # Restore original CARLA settings
        world.apply_settings(original_settings)

        print("\n" + "=" * 60)
        print(f"{'scenario':<24}{'run':<5}{'hit':<7}{'impact m/s':<12}{'min dist'}")
        for name, run, r in summary:
            if r["error"]:
                print(f"{name:<24}{run:<5}ERROR: {r['error']}")
                continue
            imp = f"{r['impact_speed']:.1f}" if r["impact_speed"] is not None else "-"
            print(f"{name:<24}{run:<5}{str(r['collided']):<7}{imp:<12}{r['min_dist']:.1f}")

        print("Simulation cleaned up")


if __name__ == "__main__":
    main()
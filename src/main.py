import math
import time
import traceback

import carla

from simulation.scenario import Scenario
from tests.csv_logger import CSVLogger
from perception.camera import Camera
from perception.detector import Detector
from tests.scenarioconfig import SCENARIOS, FIXED
from tests.scenario_builder import build_test_cases

FCW_STATES = {"WARNING"}   # whatever value of aeb.state means "FCW is showing"


AEB_ENABLED = True
REPEATS = 1
MAX_TESTS = 75
VISUAL_DELAY_SECONDS = 0.05

def print_avoidance_rate(summary):
    valid = [r for _, _, r in summary if not r["error"]]
    skipped = len(summary) - len(valid)
    if not valid:
        print("No completed runs, no avoidance rate.")
        return

    avoided = sum(1 for r in valid if not r["collided"])
    print(f"\nCollision avoidance rate: {avoided}/{len(valid)} "
          f"= {100 * avoided / len(valid):.1f}%")

    speeds = [r["impact_speed"] for r in valid
              if r["collided"] and r["impact_speed"] is not None]
    if speeds:
        print(f"Mean impact speed (collisions only): {sum(speeds)/len(speeds):.2f} m/s")
    if skipped:
        print(f"({skipped} run(s) excluded because they errored)")
def get_speed(actor):
    velocity = actor.get_velocity()

    return math.sqrt(
        velocity.x**2 +
        velocity.y**2 +
        velocity.z**2
    )

def get_crossing_distances(config, vehicle, pedestrian):
    crossing = config["crossing_point"]
    yaw = math.radians(config["yaw"])

    forward_x = math.cos(yaw)
    forward_y = math.sin(yaw)

    right_x = -math.sin(yaw)
    right_y = math.cos(yaw)

    vehicle_location = vehicle.get_location()
    pedestrian_location = pedestrian.get_location()

    # Vehicle front bumper to crossing point
    dx = crossing["x"] - vehicle_location.x
    dy = crossing["y"] - vehicle_location.y

    front_distance = (
        dx * forward_x +
        dy * forward_y -
        config["vehicle_front_offset_m"]
    )

    # Signed pedestrian lateral distance to crossing point
    ped_dx = crossing["x"] - pedestrian_location.x
    ped_dy = crossing["y"] - pedestrian_location.y

    pedestrian_crossing_distance = (
        ped_dx * right_x +
        ped_dy * right_y
    )

    return front_distance, pedestrian_crossing_distance

def run_scenario(world, config, run_number, csv_logger, aeb_enabled=True):

    result = {
        "collided": False,
        "impact_speed": None,
        "collision_frame": None,
        "min_dist": float("inf"),
        "error": None,
        "ticks": [],                    
        "termination_reason": "",       
        "vehicle_width_m": None,        
    }

    scenario = None
    camera = None
    pre_tick_speeds = {}

    try:
        # Set up scenario
        scenario = Scenario(world, config)

        if scenario.setup() is False:
            raise RuntimeError("Scenario setup failed")

        print("Scenario setup completed successfully")

        world.tick()



        vehicle_actor = scenario.vehicle.actor
        pedestrian_actor = scenario.pedestrian.actor

        front_left = vehicle_actor.get_wheel_steer_angle(
        carla.VehicleWheelLocation.FL_Wheel
        )

        front_right = vehicle_actor.get_wheel_steer_angle(
            carla.VehicleWheelLocation.FR_Wheel
        )

        vehicle_width = vehicle_actor.bounding_box.extent.y * 2
        vehicle_length = vehicle_actor.bounding_box.extent.x * 2


        result["vehicle_width_m"] = 2 * vehicle_actor.bounding_box.extent.y


        # Camera
        camera = Camera(world, vehicle_actor)
        camera.attach_camera()

        # YOLO detector
        detector = Detector(
            world,
            pedestrian_actor,
            vehicle_actor
        )

        if scenario.start() is False:
            raise RuntimeError("Scenario start failed")

        spectator = world.get_spectator()
        crossing = config["crossing_point"]

        spectator.set_transform(
            carla.Transform(
                carla.Location(
                    x=crossing["x"],
                    y=crossing["y"],
                    z=crossing["z"] + 10.0
                ),
                carla.Rotation(
                    pitch=-90.0,
                    yaw=config["yaw"],
                    roll=0.0
                )
            )
        )

        elapsed_ticks = 0

        tick_seconds = (
            world.get_settings().fixed_delta_seconds or 0.05
        )

        max_duration = config.get("max_duration", 30.0)
        max_ticks = math.ceil(max_duration / tick_seconds)

        start_time = (
            world.get_snapshot().timestamp.elapsed_seconds
        )

        crossing_start_time = None
        aeb_activation_time = None


        # Main simulation loop
        while (
            not scenario.is_complete()
            and (elapsed_ticks < max_ticks or scenario.has_collision())
        ):


            detections = []
            threat_detections = []
            path_detections = []

            pedestrian_distance = None
            ped_in_path = False

            # YOLO detection
            if camera.frame is not None:

                detections = (
                    detector.detect_pedestrians(camera.frame) or []
                )

                if detections:

                    pedestrian_distance = (
                        detector.get_pedestrian_distance()
                    )

                    for detection in detections:

                        if detector.is_in_path(detection):
                            path_detections.append(detection)

                        if detector.is_near_path(detection):
                            threat_detections.append(detection)

                    ped_in_path = bool(path_detections)
                    

            # Risk assessment, AEB, pedestrian movement
            
            scenario.update(
                threat_detections,
                pedestrian_distance,
                ped_in_path,
                aeb_enabled
            )

            # Advance simulation
            pre_tick_speed = scenario.vehicle.get_speed()
            frame_id = world.tick()
            pre_tick_speeds[frame_id] = pre_tick_speed
            elapsed_ticks += 1

            location = vehicle_actor.get_location()
            snapshot = world.get_snapshot()

            
            snapshot = world.get_snapshot()

            simulation_time = (
                snapshot.timestamp.elapsed_seconds - start_time
            )

            # Vehicle measurements
            vehicle_location = vehicle_actor.get_location()
            vehicle_speed = scenario.vehicle.get_speed()

            vehicle_control = vehicle_actor.get_control()
            acceleration = vehicle_actor.get_acceleration()

            forward = (
                vehicle_actor.get_transform().get_forward_vector()
            )

            longitudinal_acceleration = (
                acceleration.x * forward.x +
                acceleration.y * forward.y +
                acceleration.z * forward.z
            )

            # Pedestrian measurements
            pedestrian_location = pedestrian_actor.get_location()
            pedestrian_speed = get_speed(pedestrian_actor)

            # Distances
            direct_distance = vehicle_location.distance(
                pedestrian_location
            )

            ped_x_distance, ped_y_distance = (
                get_pedestrian_bumper_distances(
                    vehicle_actor,
                    pedestrian_actor
                )
            )

            (
                front_distance,
                pedestrian_crossing_distance
            ) = get_crossing_distances(
                config,
                vehicle_actor,
                pedestrian_actor
            )
            draw_crossing_distances(
            world,
            config,
            vehicle_actor,
            pedestrian_actor,
            front_distance,
            pedestrian_crossing_distance
        )

            result["min_dist"] = min(
                result["min_dist"],
                direct_distance
            )

            # Event timestamps
            if (
                scenario.crossing_started
                and crossing_start_time is None
            ):
                crossing_start_time = simulation_time

            if (
                aeb_enabled
                and vehicle_control.brake > 0
                and aeb_activation_time is None
            ):
                aeb_activation_time = simulation_time

            # Read collision status from Scenario for logging only.
            result["collided"] = scenario.has_collision()
            if result["collided"]:
                frame = scenario.collision.frame
                result["collision_frame"] = frame
                result["impact_speed"] = pre_tick_speeds.get(frame, scenario.collision.speed)
                # Align validation records with the reported collision frame.
                for tick in result["ticks"]:
                    if tick["frame"] >= frame:
                        tick["collision"] = True

            # Access current controller objects
            aeb = getattr(scenario, "aeb", None)
            risk = scenario.risk

            # Record one CSV row
            csv_logger.log_tick({
                "test_id": config.get("test_id", "unnamed-test"),
                "run_number": run_number,

                # Simulation
                "tick": elapsed_ticks,
                "simulation_time_s": simulation_time,

                # Vehicle
                "vehicle_speed_ms": vehicle_speed,
                "vehicle_acceleration_ms2": longitudinal_acceleration,
                "vehicle_brake": vehicle_control.brake,
                "vehicle_throttle": vehicle_control.throttle,
                "vehicle_steer": vehicle_control.steer,
                "vehicle_gear": vehicle_control.gear,

                "vehicle_yaw_deg": vehicle_actor.get_transform().rotation.yaw,
                "vehicle_yaw_rate_deg_s": vehicle_actor.get_angular_velocity().z,  
                "front_left_steer_deg": front_left,
                "front_right_steer_deg": front_right,   

                # Pedestrian
                "ped_speed_ms": pedestrian_speed,
                "ped_distance_to_crossing_m": pedestrian_crossing_distance,
                "ped_crossing_started": scenario.crossing_started,

                # Distances
                "vehicle_ped_distance_m": direct_distance,
                "vehicle_front_to_crossing_m": front_distance,
                "ped_longitudinal_distance_m": ped_x_distance,
                "ped_lateral_distance_m": ped_y_distance,

                # Configuration
                "target_overlap_pct": config.get("overlap"),
                "vehicle_target_speed_kmh": config.get("vehicle_speed"),
                "ped_target_speed_kmh": config.get("pedestrian_speed"),

                # Perception
                "ped_detected": bool(detections),
                "detection_confidence": (
                    max(
                        detection.get("confidence", 0)
                        for detection in detections
                    )
                    if detections else None
                ),
                "ped_in_path": ped_in_path,

                # Risk assessment

                "ttc_s": getattr(risk, "last_ttc", None),
                "ttc_brake_s": getattr(risk, "last_ttc_brake", None),
                "risk_state": getattr(risk, "last_risk_state", None),

                # AEB
                "aeb_state": getattr(aeb, "state", None),
                "required_deceleration_ms2": getattr(
                    aeb, "last_required_deceleration", None
                ),
                "brake_command": vehicle_control.brake,

                # Collision
                "collision_occurred": result["collided"]
            })

            result["ticks"].append({
                "t": simulation_time,
                "vehicle_x": vehicle_location.x,
                "vehicle_y": vehicle_location.y,
                "vehicle_speed": vehicle_speed,                       # m/s
                "vehicle_yaw": vehicle_actor.get_transform().rotation.yaw,
                "throttle": vehicle_control.throttle,
                "manual_brake": 0.0,                                  # see note below
                "ped_x": pedestrian_location.x,
                "ped_y": pedestrian_location.y,
                "ped_speed": pedestrian_speed,                        # m/s
                "fcw_active": getattr(aeb, "state", None) in FCW_STATES,
                "collision": scenario.has_collision(),
                "frame": frame_id,
            })

            # Display camera
            camera.display(detector.path_polygon)

            if VISUAL_DELAY_SECONDS:
                time.sleep(VISUAL_DELAY_SECONDS)


        last = result["ticks"][-1] if result["ticks"] else None
        if result["collided"]:
            result["termination_reason"] = "collision"
        elif elapsed_ticks >= max_ticks:
            result["termination_reason"] = "timeout"
        elif last and last["vehicle_speed"] <= 0.05:
            result["termination_reason"] = "stopped"
        else:
            result["termination_reason"] = "pedestrian_cleared"

        if elapsed_ticks >= max_ticks and not result["collided"]:
            print(f"Reached test duration limit ({max_duration} s)")

    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
        print(f"Test failed: {result['error']}")
        traceback.print_exc()

    finally:
        

        if camera is not None:
            camera.destroy()

        if scenario is not None:
            scenario.cleanup()

        world.tick()

    return result

def get_pedestrian_bumper_distances(vehicle_actor, pedestrian_actor):

    # Get the vehicle's current position and orientation
    transform = vehicle_actor.get_transform()

    # Get the vehicle's bounding box
    box = vehicle_actor.bounding_box

    # Get the pedestrian's world location
    pedestrian_location = pedestrian_actor.get_location()

    # Convert the pedestrian's world location into
    # coordinates relative to the vehicle
    local_pedestrian = transform.inverse_transform(
        pedestrian_location
    )

    # Calculate the front bumper's local X position
    front_bumper_x = box.location.x + box.extent.x

    # Calculate the front bumper's lateral center
    front_bumper_y = box.location.y

    # Calculate the forward/backward distance
    x_distance = local_pedestrian.x - front_bumper_x

    # Calculate the left/right distance
    y_distance = local_pedestrian.y - front_bumper_y

    return x_distance, y_distance

def draw_crossing_distances(
    world,
    config,
    vehicle_actor,
    pedestrian_actor,
    front_distance,
    pedestrian_crossing_distance
):

    crossing = config["crossing_point"]

    # Height above road so lines are visible
    z = crossing["z"] + .15

    crossing_location = carla.Location(
        x=crossing["x"],
        y=crossing["y"],
        z=z
    )

    # Get front bumper location
    transform = vehicle_actor.get_transform()
    box = vehicle_actor.bounding_box

    bumper_location = transform.transform(
        carla.Location(
            x=box.location.x + box.extent.x,
            y=box.location.y
        )
    )

    bumper_location.z = z

    # Get pedestrian location
    ped_location = pedestrian_actor.get_location()
    ped_location.z = z

    # Drawing settings
    red = carla.Color(255, 0, 0)
    blue = carla.Color(0, 170, 255)
    white = carla.Color(255, 255, 255)

    life_time = 0.025

    # Draw crossing point
    world.debug.draw_hud_point(
        crossing_location,
        size=6,
        color=white,
        life_time=life_time
    )
    # Vehicle distance line
    world.debug.draw_hud_line(
        bumper_location,
        crossing_location,
        thickness=3,
        color=red,
        life_time=life_time
    )

    # Pedestrian distance line
    world.debug.draw_hud_line(
        ped_location,
        crossing_location,
        thickness=3,
        color=blue,
        life_time=life_time
    )


    # Midpoint of vehicle line
    vehicle_midpoint = carla.Location(
        x=(bumper_location.x + crossing_location.x) / 2,
        y=(bumper_location.y + crossing_location.y) / 2,
        z=z + 0.3
    )

    # Midpoint of pedestrian line
    pedestrian_midpoint = carla.Location(
        x=(ped_location.x + crossing_location.x) / 2,
        y=(ped_location.y + crossing_location.y) / 2,
        z=z + 0.3
    )

    # Vehicle distance label
    world.debug.draw_string(
        vehicle_midpoint,
        f"{front_distance:.2f} m",
        draw_shadow=True,
        color=red,
        life_time=life_time
    )

    # Pedestrian distance label
    world.debug.draw_string(
        pedestrian_midpoint,
        f"{pedestrian_crossing_distance:.2f} m",
        draw_shadow=True,
        color=blue,
        life_time=life_time
    )

    vehicle_width = config["vehicle_width"]

    yaw_rad = math.radians(config["yaw"])

    right_x = -math.sin(yaw_rad)
    right_y = math.cos(yaw_rad)

    for overlap, color in [
        (0.25, carla.Color(255, 165, 0)),
        (0.50, carla.Color(0, 255, 0))
    ]:

        lateral_offset = (
            vehicle_width / 2
            - overlap * vehicle_width
        )

        overlap_point = carla.Location(
            x=config["crossing_point"]["x"] + lateral_offset * right_x,
            y=config["crossing_point"]["y"] + lateral_offset * right_y,
            z=config["crossing_point"]["z"] + 0.15
        )

        world.debug.draw_hud_point(
            overlap_point,
            size=10,
            color=color,
            life_time=0.06
        )

        world.debug.draw_string(
            overlap_point + carla.Location(z=0.3),
            f"{overlap * 100:.0f}%",
            draw_shadow=False,
            color=color,
            life_time=0.05,
            persistent_lines=False
        )

def main():

    client = carla.Client("localhost", 2000)
    client.set_timeout(20.0)

    target_town = FIXED.get("town", "Town04")

    world = client.get_world()

    current_town = world.get_map().name.split("/")[-1]

    if current_town != target_town:
        print(f"Loading {target_town}...")
        world = client.load_world(target_town)

    print("Connected to CARLA; map:", world.get_map().name)

    # Generate test cases
    all_cases = build_test_cases(SCENARIOS, FIXED)

    crossing_cases = [
        config for config in all_cases
        if config["scenario"] == "stationary"

    
    
    ]

    if MAX_TESTS is not None:
        crossing_cases = crossing_cases[:MAX_TESTS]

    print(f"Running {len(crossing_cases)} crossing-road test case(s)")

    original_settings = world.get_settings()
    
    
    summary = []
    csv_logger = CSVLogger()

    try:
        settings = world.get_settings()

        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        settings.substepping = True
        settings.max_substep_delta_time = 0.01
        settings.max_substeps = 10
        settings.no_rendering_mode = False

        world.apply_settings(settings)

        settings = world.get_settings()


        for config in crossing_cases:

            for run_number in range(1, REPEATS + 1):

                test_id = config.get("test_id", "unnamed-test")

                print(
                    f"\n=== {test_id} "
                    f"(run {run_number}/{REPEATS}) ==="
                )

                print(
                    "Vehicle speed:",
                    config.get("vehicle_speed"),
                    "km/h"
                )

                print(
                    "Pedestrian speed:",
                    config.get("pedestrian_speed"),
                    "km/h"
                )

                result = run_scenario(
                    world,
                    config,
                    run_number,
                    csv_logger,
                    AEB_ENABLED
                )
                summary.append((test_id, run_number, result))
                csv_logger.end_run()

    finally:
        
        csv_logger.close()
        print("CSV saved:", csv_logger.filepath)
        world.apply_settings(original_settings)

        print("\n" + "=" * 75)
        print(
            f"{'Test':<38} {'Run':<5} "
            f"{'Hit':<7} {'Impact m/s':<12} {'Min dist (m)'}"
        )

        for test_id, run_number, result in summary:

            if result["error"]:
                print(
                    f"{test_id:<38} {run_number:<5} "
                    f"ERROR: {result['error']}"
                )
            else:
                impact = (
                    f"{result['impact_speed']:.2f}"
                    if result["impact_speed"] is not None
                    else "-"
                )

                min_dist = (
                    f"{result['min_dist']:.2f}"
                    if math.isfinite(result["min_dist"])
                    else "-"
                )

                print(
                    f"{test_id:<38} {run_number:<5} "
                    f"{str(result['collided']):<7} "
                    f"{impact:<12} {min_dist}"
                )

        print_avoidance_rate(summary)
        print("Simulation cleaned up")


if __name__ == "__main__":
    main()

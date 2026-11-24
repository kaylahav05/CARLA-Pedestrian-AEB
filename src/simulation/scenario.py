# this file to generate vehicle and pedestrian scenarios
from simulation.pedestrian import Pedestrian
from simulation.vehicle import Vehicle
from control.aeb_system import AEB
from control.risk_assessment import RiskAssessment
from perception.detector import Detector
from perception.collision_detector import CollisionDetector
import carla
import math
import time

class Scenario:
    def __init__(self, world, config):
        self.world = world
        self.config = config

        self.vehicle = None
        self.pedestrian = None
        self.is_running = False
        self.crossing_started = False
        self.crossing_finished = False
        self.start_time = None
        self.end_time = None
        self.obstructions = []

        self.risk = None
        self.aeb = None
        self.collision = None
        self.pedestrian_was_ahead = False
        


    def calculate_initial_headway(self):
        ttc0 = self.config["initial_ttc"]
        V_sv = self.config["vehicle_speed"] / 3.6 # convert to m/s

        if self.config["scenario"] == "along-path":
            V_py = self.config["pedestrian_speed"] / 3.6

        else:
            V_py = 0 # longitudal component


        return ttc0 * (V_sv - V_py)

    def calculate_vehicle_spawn(self):
        """
        # calculates where the vehicle should spawn based on 
        # initial cross point and headway

        # headway between the front plane of the vechicle &
        # and a parallet contact plane

        # direction towards the crossing point

        # returns a carla transform
        """

        # get crossing point, yaw, and headway
        crossing_point = self.config["crossing_point"]
        yaw = self.config["yaw"]
        init_headway = self.calculate_initial_headway()

        # account for vehicle bumper
        front_offset = self.config["vehicle_front_offset_m"]

        # determine actual position
        RUN_UP_S = 1.5   # seconds of settling time before L0
        run_up = (self.config["vehicle_speed"] / 3.6) * RUN_UP_S
        spawn_distance = init_headway + front_offset + run_up

        yaw_rad = math.radians(yaw)
        spawn_x = crossing_point["x"] - spawn_distance * math.cos(yaw_rad)
        spawn_y = crossing_point["y"] - spawn_distance * math.sin(yaw_rad)

        return carla.Transform(
            carla.Location(
                x=spawn_x,
                y=spawn_y,
                z=crossing_point["z"]
            ),
            carla.Rotation(yaw=yaw)
        )

    def calculate_pedestrian_spawn(self):
        crossing_point = self.config["crossing_point"]
        yaw = self.config["yaw"]
        yaw_rad = math.radians(yaw)

        if self.config["scenario"] == "crossing-road":
            offset = self.config["pedestrian_start_offset"]

            if self.config["direction"] == "right-left":
                lateral_offset = offset
                pedestrian_yaw = yaw - 90

            elif self.config["direction"] == "left-right":
                lateral_offset = -offset
                pedestrian_yaw = yaw + 90

            else:
                raise ValueError("Invalid crossing direction")

            # Calculate right direction relative to vehicle
            right_x = -math.sin(yaw_rad)
            right_y = math.cos(yaw_rad)

            # Calculate pedestrian spawn coordinates
            spawn_x = crossing_point["x"] + lateral_offset * right_x
            spawn_y = crossing_point["y"] + lateral_offset * right_y

            return carla.Transform(
                carla.Location(
                    x=spawn_x,
                    y=spawn_y,
                    z=crossing_point["z"]
                ),
                carla.Rotation(yaw=pedestrian_yaw)
            )

        elif self.config["scenario"] in ("stationary", "along-path"):
            vehicle_width = self.config["vehicle_width"]
            overlap = self.config["overlap"]

            # Calculate 25% right-overlap position
            overlap_distance = overlap * vehicle_width
            lateral_offset = (vehicle_width / 2) - overlap_distance

            # Calculate pedestrian spawn coordinates
            spawn_x = crossing_point["x"] - lateral_offset * math.sin(yaw_rad)
            spawn_y = crossing_point["y"] + lateral_offset * math.cos(yaw_rad)

            return carla.Transform(
                carla.Location(
                    x=spawn_x,
                    y=spawn_y,
                    z=crossing_point["z"]
                ),
                carla.Rotation(yaw=yaw)
            )

        else:
            raise ValueError(
                f"Unknown scenario: {self.config['scenario']}"
            )

    def config_lighting(self):
        weather = self.world.get_weather()

        weather.cloudiness = 0.0
        weather.precipitation = 0.0
        weather.precipitation_deposits = 0.0
        weather.wetness = 0.0
        weather.fog_density = 0.0
        weather.fog_distance = 0.0
        weather.fog_falloff = 0.0
        weather.dust_storm = 0.0

        if self.config["lighting"] == "day":
            weather.sun_altitude_angle = 70
        else:
            weather.sun_altitude_angle = -30
            if self.config["lighting"] == "low-beam":
                light_state = carla.VehicleLightState.LowBeam
            elif self.config["lighting"] == "high-beam":
                light_state = carla.VehicleLightState.HighBeam
                lm = self.world.get_lightmanager()
                lm.turn_off(lm.get_all_lights(carla.LightGroup.Street))
 
            self.vehicle.actor.set_light_state(light_state)
        self.world.set_weather(weather)
   
    def place_obstructions(self):

        # Get the crossing point and intended direction of travel
        crossing_point = self.config["crossing_point"]
        yaw = self.config["yaw"]
        yaw_rad = math.radians(yaw)

        # Get half the width of the ego vehicle
        ego_half_width = self.vehicle.actor.bounding_box.extent.y

        # Get obstruction vehicle dimensions
        obstruction_half_width = self.config["obstruction_width"] / 2
        obstruction_half_length = self.config["obstruction_length"] / 2

        # Required distances between vehicles
        lateral_gap = 1.0
        longitudinal_gap = 1.0
        crossing_gap = 1.0

        # Calculate the lateral distance from the intended travel
        # path to the center of each obstruction vehicle.
        # This creates a 1-meter gap between the vehicle edges.
        lateral_offset = (
            ego_half_width
            + lateral_gap
            + obstruction_half_width
        )

        # Position the front obstruction so its front bumper
        # is 1 meter before the pedestrian crossing point.
        front_offset = -(
            crossing_gap
            + obstruction_half_length
        )

        # Position the rear obstruction directly behind the front
        # obstruction with a 1-meter bumper-to-bumper gap.
        rear_offset = front_offset - (
            2 * obstruction_half_length
            + longitudinal_gap
        )

        # Get the blueprint for the obstruction vehicles
        blueprint = self.world.get_blueprint_library().find(
            self.config["obstruction_blueprint"]
        )

        # Calculate the spawn location for each obstruction
        for longitudinal_offset in [front_offset, rear_offset]:

            # Calculate world coordinates using the intended
            # direction of travel and the calculated offsets.
            spawn_x = (
                crossing_point["x"]
                + longitudinal_offset * math.cos(yaw_rad)
                - lateral_offset * math.sin(yaw_rad)
            )

            spawn_y = (
                crossing_point["y"]
                + longitudinal_offset * math.sin(yaw_rad)
                + lateral_offset * math.cos(yaw_rad)
            )

            # Create the spawn transform.
            # Both obstructions face the same direction as the ego vehicle.
            spawn_transform = carla.Transform(
                carla.Location(
                    x=spawn_x,
                    y=spawn_y,
                    z=crossing_point["z"]
                ),
                carla.Rotation(yaw=yaw)
            )

            # Spawn the obstruction directly at its final position
            obstruction = self.world.spawn_actor(
                blueprint,
                spawn_transform
            )

            # Disable physics to keep the obstruction stationary
            obstruction.set_simulate_physics(False)

            # Store the obstruction for validation and cleanup
            self.obstructions.append(obstruction)

    # initialize actors and spawn them config initial spawns
    def setup(self):


        # vehicle set up
        self.vehicle = Vehicle(self.world)
        vehicle_spawn = self.calculate_vehicle_spawn()

        print("In scenario setup: attempting to spawn vehicle at ")
        print(vehicle_spawn)
        print()

        if not self.vehicle.spawn(self.config["vehicle_id"], 
                           vehicle_spawn):
            print("Scenario setup failed: cant spawn vehicle")
            self.cleanup()               
            return False 

        self.vehicle.set_target_speed(self.config["vehicle_speed"])

        waypoint = self.world.get_map().get_waypoint(
            vehicle_spawn.location
        )


        # pedestrian set up
        self.pedestrian = Pedestrian(self.world)
        pedestrian_spawn = self.calculate_pedestrian_spawn()
        
        if not self.pedestrian.spawn(self.config["pedestrian_id"], 
                              pedestrian_spawn):
            print("Scenario setup failed: cant spawn pedestrian")
            self.cleanup()               
            return False 

        self.pedestrian.set_speed(self.config["pedestrian_speed"])
        # TO DO: check if obstructed and set up accordingly

        # light set up
        self.config_lighting()

        # Only place obstruction vehicles for obstructed-right scenarios.
        if self.config["variation"] == "obstructed-right":
            self.place_obstructions()

        # AEB and risk set up
        self.risk = RiskAssessment()
        self.aeb = AEB(self.vehicle)

        self.collision = CollisionDetector(
            self.world, self.vehicle.actor, self.pedestrian.actor
        )
        self.collision.attach()

        return True
        
    # begin vehicle and pedestrian movement
    def start(self):
        #check that actors exist
        if self.vehicle is None or self.pedestrian is None:
            print("Cannot start scenario: actors not initialized")
            return False

        if self.vehicle.actor is None or self.pedestrian.actor is None:
            print("Cannot start scenario: actors not spawned")
            return False

        #check if the scenario is already running
        if self.is_running:
            print("Scenario is already running")
            return False

        # record start time & start
        self.start_time = self.world.get_snapshot().timestamp.elapsed_seconds
        self.is_running = True

        vehicle_speed = self.config["vehicle_speed"]
        vehicle_speed_ms = vehicle_speed / 3.6
        yaw = self.config["yaw"]
        yaw_rad = math.radians(yaw)

        vehicle_velocity = carla.Vector3D(
            x=vehicle_speed_ms * math.cos(yaw_rad),
            y=vehicle_speed_ms * math.sin(yaw_rad),
            z=0.0
        )
        self.vehicle.actor.set_target_velocity(vehicle_velocity)


        return True
    
    def calculate_crossing_trigger_distance(self):

        vehicle_speed = self.config["vehicle_speed"] / 3.6
        pedestrian_speed = self.config["pedestrian_speed"] / 3.6

        vehicle_width = self.config["vehicle_width"]
        overlap = self.config["overlap"]
        start_offset = self.config["pedestrian_start_offset"]

        # target distance from vehicle centerline
        target_offset = (vehicle_width / 2) - (overlap * vehicle_width)

        crossing_distance = start_offset - target_offset
        # Compensate for observed pedestrian movement delay
        movement_delay = 0.35

        crossing_time = (
            crossing_distance / pedestrian_speed
            + movement_delay
        )

        # distance from vehicle front bumper to crossing point
        trigger_distance = vehicle_speed * crossing_time

        return trigger_distance
    

        return points
    # check if the pedestrian should start crossing the road yet
    def should_start_crossing(self):
        vehicle_location = self.vehicle.get_location()

        if vehicle_location is None:
            return False

        crossing_point = self.config["crossing_point"]
        yaw_rad = math.radians(self.config["yaw"])

        # Distance from vehicle center to crossing point
        dx = crossing_point["x"] - vehicle_location.x
        dy = crossing_point["y"] - vehicle_location.y

        distance = dx * math.cos(yaw_rad) + dy * math.sin(yaw_rad)

        # Distance from front bumper to crossing point
        front_distance = distance - self.config["vehicle_front_offset_m"]

        # Calculate when pedestrian should begin crossing
        trigger_distance = self.calculate_crossing_trigger_distance()

        return front_distance <= trigger_distance    
    
    def has_collision(self):
        return self.collision is not None and self.collision.detected
    # update behavior with simulation ticks
    def update(self, detections, distance_between, ped_in_path, aeb_enabled):
        if not self.is_running:
            print("Cant update scenario, it isnt running")
            return
        self.collision.update()

        # get info for risk assessment
        vehicle_speed = self.vehicle.get_speed()
        pedestrian_speed = self.pedestrian.get_speed_ms()

        if detections:
            confidence = detections[0]["confidence"]
        else:
            confidence = 0.0

        # Assess risk
        if distance_between is not None:
            risk_state = self.risk.assess_risk(
                confidence,
                vehicle_speed,
                pedestrian_speed,
                distance_between,

            )
        else:
            risk_state = "NONE"


        # once AEB is set up, set control to aeb control if necessary
        if aeb_enabled:
            self.aeb.update(risk_state, ped_in_path, vehicle_speed, distance_between)

            if (self.aeb.state != "EMERGENCY_BRAKE" and self.aeb.state != "STOPPED" ):
                self.vehicle.maintain_speed()

            ## add display for warning
        else: 
            self.vehicle.maintain_speed()

        if self.config["scenario"] == "crossing-road":
            # check if ped should start crossing & move pedestrian
            if not self.crossing_started:
                if self.should_start_crossing():
                    self.crossing_started = True

            if self.crossing_started and not self.crossing_finished:
                moving = self.pedestrian.move_pedestrian()

                if not moving:
                    self.crossing_finished = True
        elif self.config["scenario"] == "along-path":

            if not self.crossing_started:

                vehicle_location = self.vehicle.get_location()
                pedestrian_location = self.pedestrian.actor.get_location()

                yaw_rad = math.radians(self.config["yaw"])

                dx = pedestrian_location.x - vehicle_location.x
                dy = pedestrian_location.y - vehicle_location.y

                headway = (
                    dx * math.cos(yaw_rad)
                    + dy * math.sin(yaw_rad)
                    - self.config["vehicle_front_offset_m"]
                )

                if headway <= self.calculate_initial_headway():
                    self.crossing_started = True

            if self.crossing_started:
                self.pedestrian.move_pedestrian()

    
    def has_passed_pedestrian(self):
        vehicle = self.vehicle.actor
        pedestrian = self.pedestrian.actor

        vehicle_transform = vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        pedestrian_location = pedestrian.get_location()

        forward = vehicle_transform.get_forward_vector()

        dx = pedestrian_location.x - vehicle_location.x
        dy = pedestrian_location.y - vehicle_location.y

        distance_ahead = dx * forward.x + dy * forward.y

        # Track whether the pedestrian has been ahead of the vehicle
        if distance_ahead > 0:
            self.pedestrian_was_ahead = True

        # Only count as passed if previously ahead
        return (
            self.pedestrian_was_ahead
            and distance_ahead < -5.0
        )
   
    # determines if the scenario has ended
    def is_complete(self):
        
        # if a collision occurs return true
        if self.has_collision():
            print("ended bc collison detected")
            time.sleep(2)
            return self.has_collision()

        if self.config["scenario"] == "stationary" or self.config["scenario"] == "along-path":
            if self.aeb.state == "STOPPED":
                return True

        # if vehicle passed the pedestrian
        if (self.has_passed_pedestrian()):
            print("Ended bc has passed pedestrian")
            self.is_running = False
            return True

        
        # check if maxed time reached
        current_time = self.world.get_snapshot().timestamp.elapsed_seconds
        elapsed_time = current_time - self.start_time
        if (elapsed_time >= self.config["max_duration"]):
            print("Ended bc max duration")
            self.is_running = False
            return True

        return False

    def cleanup(self):

        if self.collision is not None:
            self.collision.destroy()

        # Stop scenario
        self.is_running = False

        # Destroy pedestrian
        if self.pedestrian is not None:
            self.pedestrian.destroy()
            self.pedestrian = None

        # Destroy vehicle
        if self.vehicle is not None:
            self.vehicle.destroy()
            self.vehicle = None

        for obstruction in self.obstructions:
            if obstruction.is_alive:
                        obstruction.destroy()

        self.obstructions.clear()

    

        # Reset scenario state
        self.crossing_started = False
        self.crossing_finished = False
        self.start_time = None

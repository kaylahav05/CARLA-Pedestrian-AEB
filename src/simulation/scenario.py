# this file to generate vehicle and pedestrian scenarios
from simulation.pedestrian import Pedestrian
from simulation.vehicle import Vehicle
import carla

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

    # initialize actors and spawn them config initial spawns
    def setup(self):

        # vehicle set up
        self.vehicle = Vehicle(self.world)

        if not self.vehicle.spawn(self.config["vehicle_id"], 
                           self.config["vehicle_spawn"]):
            print("Scenario setup failed: cant spawn vehicle")
            self.cleanup()               
            return False #

        self.vehicle.set_destination(self.config["vehicle_destination"])
        self.vehicle.set_target_speed(self.config["vehicle_target_speed"])



        # pedestrian set up
        self.pedestrian = Pedestrian(self.world)
        
        if not self.pedestrian.spawn(self.config["pedestrian_id"], 
                              self.config["pedestrian_spawn"]):
            print("Scenario setup failed: cant spawn pedestrian")
            self.cleanup()               
            return False 

        self.pedestrian.set_destination(self.config["pedestrian_destination"])
        self.pedestrian.set_speed(self.config["pedestrian_speed"])
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

        return True

    # check if the pedestrian should start crossing the road yet
    def should_start_crossing(self):
        # get vehicles current location
        vehicle_location = self.vehicle.get_location() # need error handling

        if vehicle_location is None:
            return False
        
        # get the crossing location (where collision would happen)
        crossing_location = self.config["pedestrian_spawn"].location
        trigger_distance = self.config["crossing_trigger_distance"]

        # calculate distance & compare with trigger distance
        distance = vehicle_location.distance(crossing_location)

        # if vehicle is close enough, return true
        return distance <= trigger_distance

    # update behavior with simulation ticks
    def update(self):
        if not self.is_running:
            print("Cant update scenario, it isnt running")
            return

        # get controls from basic agent & apply them
        control = self.vehicle.get_agent_control()

        # once AEB is set up, set control to aeb control if necessary

        self.vehicle.apply_control(control)

        # check if ped should start crossing & move pedestrian
        if not self.crossing_started:
            if self.should_start_crossing():
                self.crossing_started = True

        if self.crossing_started and not self.crossing_finished:
            moving = self.pedestrian.move_pedestrian()

            if not moving:
                self.crossing_finished = True

    # determines if the scenario has ended
    def is_complete(self):

        planner = self.vehicle.agent.get_local_planner()

        if (self.vehicle.reached_destination()):
            print("Scenario ended: vehicle reached destination")
            self.is_running = False
            return True
        # if a collision occurs return true

        # check if maxed time reached
        current_time = self.world.get_snapshot().timestamp.elapsed_seconds
        elapsed_time = current_time - self.start_time
        if (elapsed_time >= self.config["max_duration"]):
            print("Ended bc max duration")
            self.is_running = False
            return True

        return False

    def cleanup(self):
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

        # Reset scenario state
        self.crossing_started = False
        self.crossing_finished = False
        self.start_time = None

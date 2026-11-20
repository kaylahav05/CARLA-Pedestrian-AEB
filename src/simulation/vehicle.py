
# This file is for spawning and controlling the vehicles

import carla
import math
import random 
from agents.navigation.basic_agent import BasicAgent

class Vehicle:

    def __init__(self, world): # constructor
        self.world = world
        self.actor = None
        self.spawn_transform = None
        self.destination = None
        self.target_speed = None
        self.agent = None

    # spawns the vehicle in the world with specific blueprint and location
    def spawn(self, blueprint_id, spawn_transform):
        # make sure it wasnt already spawned yet
        if self.actor is not None:
            print("Cannot spawn: vehicle already exists")
            return False
    
        # select vehicle model
        blueprint_library = self.world.get_blueprint_library() # returns list of blueprints
        try:
            if not blueprint_id.startswith("vehicle."):
                print(f"Invalid vehicle ID: {blueprint_id}")
                return False
            blueprint = blueprint_library.find(blueprint_id) 
            
        except IndexError:
            print(f"Vehicle blueprint '{blueprint_id}' not found.")
            return False

        # select vehicle position and orientation
        #  -- this random location thing will be gone eventually
        if spawn_transform is None:
            self.spawn_transform = random.choice(self.world.get_map().get_spawn_points())
        else:
            self.spawn_transform = spawn_transform

        # spawn the ego vehicle & store actor reference
        try:
            self.actor = self.world.spawn_actor(blueprint, self.spawn_transform) # implement error handing
            return True

        except RuntimeError as e:
            print(f"Failed to spawn vehicle: {e}")
            self.actor = None
            return False

    def set_destination(self, destination):
        self.destination = destination
        if self.agent is not None:
            self.agent.set_destination(destination)

    def set_target_speed(self, target_speed):
        self.target_speed = target_speed
        if self.agent is not None:
            self.agent.set_target_speed(target_speed)

    def setup_agent(self):
        if self.agent is not None:
            print("BasicAgent is already initialized")
            return False

        if self.actor is None:
            print("Cannot initialize BasicAgent: vehicle does not exist")
            return False

        if self.target_speed is None or self.destination is None:
            print("Target speed and destination must be set first")
            return False

        self.agent = BasicAgent(self.actor, target_speed=self.target_speed)
        self.agent.set_destination(self.destination)
        return True

    # returns the current location of the vehicle
    def get_location(self):
        if self.actor is None:
            print("Cannot get location: vehicle does not exist")
            return None

        location = self.actor.get_location()
        return location
    
    # returns the current speed of the vehicle
    def get_speed(self):

        if self.actor is None:
            print("Cannot get speed: vehicle does not exist")
            return None

        velocity = self.actor.get_velocity()

        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        print(f"Vehicle speed: {speed:.2f} m/s")

        return speed

    # gets basic agents reccommended driving command
    def get_agent_control(self):
        if self.agent is None:
            print("BasicAgent has not been initialized")
            return None

        return self.agent.run_step()

    # applies the controls to the vehicle -> throttle steer & brake
    def apply_control(self, control):
        # check the actor exists
        if self.actor is None:
            print("Cannot apply control: The vehicle object does not exist")
            return False
        if control is None:
            print("Cannot apply control: no control provided")
            return False

        # apply the control
        self.actor.apply_control(control)
        return True

    # checks if the vehicle has reached its destination
    def reached_destination(self):
        # check the agent exists
        if self.agent is None:
            print("Cant check reached destination:BasicAgent has not been initialized")
            return False

        return self.agent.done()

    # removes the vehicle from the world, clears the reference
    #  *** TO DO: remove sensors first
    def destroy(self):
        # check the actor exists
        if self.actor is None:
                print("The vehicle object does not exist")
                return

        # destroy and clear
        self.actor.destroy()
        self.actor = None
        self.agent= None
        self.spawn_transform = None
        self.destination = None
        self.target_speed = None
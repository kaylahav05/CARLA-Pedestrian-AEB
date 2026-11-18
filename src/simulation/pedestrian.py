
# This file is for spawning and controlling the pedestrians

import carla
import random

class Pedestrian:

    def __init__(self, world): # constructor
        self.world = world
        self.actor = None
        self.spawn_transform = None
        self.destination = None
        self.speed = None

    def spawn(self, blueprint_id, spawn_transform):

        if self.actor is not None:
            print("Cannot spawn: pedestrian already exists")
            return False
       
        # select pedestrian model
        """ PEDESTRIAN BLUEPRINTS
                skinny adult women
                skinny adult men
                fat adult women
                fat adult men
                children
                police
                some variants can use wheelchair <- later thing
        """
        blueprint_library = self.world.get_blueprint_library() # returns list of blueprints
        try:
            if not blueprint_id.startswith("walker.pedestrian."):
                print(f"Invalid pedestrian ID: {blueprint_id}")
                return False
            blueprint = blueprint_library.find(blueprint_id) 
            
        except IndexError:
            print(f"Pedestrian blueprint '{blueprint_id}' not found.")
            return False

        # select pedestrian position and orientation
        # --- random spawn to be removed
        if spawn_transform is None:
            location = self.world.get_random_location_from_navigation()

            if location is None:
                print("No valid pedestrian spawn location found")
                return False

            self.spawn_transform = carla.Transform(location)

        else:
            self.spawn_transform = spawn_transform

        # spawn the pedestrian & store actor reference
        try:
            self.actor = self.world.spawn_actor(blueprint, self.spawn_transform)
            return True
        except RuntimeError as e:
            print(f"Failed to spawn pedestrian: {e}")
            self.actor = None
            return False

    def set_destination(self, destination):
        self.destination = destination

    def set_speed(self, speed):
        self.speed = speed

    def move_pedestrian(self):
        # check that pedestrian exists
        if self.actor is None:
            return False

        # check that destination and speed are set
        if self.destination is None or self.speed is None:
            return False

        if self.speed <= 0:
            return False

        # get current pedestrian position
        current = self.actor.get_location()

        # calculate direction toward destination
        direction = self.destination - current
        direction.z = 0.0

        # check if destination reached
        if direction.length() <= 0.5:
            self.actor.apply_control(carla.WalkerControl(speed=0.0))
            return False

        # normalize direction
        direction = direction.make_unit_vector()

        # move pedestrian
        control = carla.WalkerControl(
            direction=direction,
            speed=self.speed,
            jump=False
        )

        self.actor.apply_control(control)

        return True

    def get_location(self):
        if self.actor is None:
                print("Cannot get location: pedestrian does not exist")
                return None
        
        location = self.actor.get_location()
        return location

    def destroy(self):
        # check the actor exists
        if self.actor is None:
                print("The pedestrian object does not exist")
                return

        # destroy and clear
        self.actor.destroy()
        self.actor = None
        self.spawn_transform = None
        self.destination= None
        self.speed = None


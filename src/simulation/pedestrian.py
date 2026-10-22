
# This file is for spawning and controlling the pedestrians

import carla
import random

class Pedestrian:

    def __init__(self, world): # constructor
        self.world = world
        self.actor = None

    def spawn(self, id):
        print("in ped spawn function")
       
        """ select pedestrian model
            PEDESTRIAN BLUEPRINTS
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
            if not id.startswith("walker.pedestrian."):
                print(f"Invalid pedestrian ID: {id}")
                return
            blueprint = blueprint_library.find(id) 
            
        except IndexError:
            print(f"Pedestrian blueprint '{id}' not found.")
            return

        # select vehicle position and orientation
        # For now, spawn points are hardcoded carla spawn points for simiplicity
        # Will be changed later
        locations = []

        for _ in range(50):
            location = self.world.get_random_location_from_navigation()

            if location is not None:
                locations.append(location)

        location = random.choice(locations)
        transform = carla.Transform(location)

        # spawn the pedestrian & store actor reference
        try:
            self.actor = self.world.spawn_actor(blueprint, transform) # implement error handing

        except RuntimeError as e:
            print(f"Failed to spawn pedestrian: {e}")
            self.actor = None

    def move_pedestrian(self, x, y, z, speed):
        # check the actor exists
        if self.actor is None:
            print("The pedestrian object does not exist")
            return
        
        control = carla.WalkerControl(
            direction = carla.Vector3D(x,y,z),
            speed = speed
        )

        self.actor.apply_control(control)

    def reset_pedestrian(self):
        if self.actor is None:
            print("The pedestrian object does not exist")
            return

        # stop movement
        control = carla.WalkerControl(
                    direction=carla.Vector3D(0,0,0),
                    speed= 0
                )
        self.actor.apply_control(control)

        spawn_points = self.world.get_map().get_spawn_points()
        transform = spawn_points[0]
        self.actor.set_transform(transform)

    def destroy(self):
        # check the actor exists
        if self.actor is None:
                print("The pedestrian object does not exist")
                return

        # destroy and clear
        self.actor.destroy()
        self.actor = None

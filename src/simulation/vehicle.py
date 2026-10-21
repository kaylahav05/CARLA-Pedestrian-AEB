
# This file is for spawning and controlling the vehicles

import carla

class Vehicle:

    def __init__(self, world): # constructor
        self.world = world
        self.actor = None
        self.spawned = False

    def spawn(self, id):
       
        # select vehicle model
        blueprint_library = self.world.get_blueprint_library() # returns list of blueprints
        try:
            if not id.startswith("vehicle."):
                print(f"Invalid vehicle ID: {id}")
                return
            blueprint = blueprint_library.find(id) 
            
        except IndexError:
            print(f"Vehicle blueprint '{id}' not found.")
            return

        # select vehicle position and orientation
        spawn_points = self.world.get_map().get_spawn_points()
        transform = spawn_points[0]

        # spawn the ego vehicle & store actor reference
        try:
            self.actor = self.world.spawn_actor(blueprint, transform) # implement error handing
            self.spawned = True

        except RuntimeError as e:
            print(f"Failed to spawn vechile: {e}")
            self.spawned = False
            self.actor = None

    # control the vehicle
    def apply_control(self, throttle, steer, brake):
        # check the actor exists
        if self.actor is None:
            print("The vehicle object does not exist")
            return

        # apply the control
        self.actor.apply_control(carla.VehicleControl(throttle, steer, brake))

        print(f"Speed: {self.actor.get_velocity()}")

    # return vehicle to the original position - NOT DONE to be finished later & tested
    def reset_vehicle(self):
        if self.actor is None:
            print("The vehicle object does not exist")
            return

        # reset its velocity so its not moving
        self.actor.set_target_velocity(carla.Vector3D(0, 0, 0))
        self.actor.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
        self.apply_control(0.0,0.0,0.0)
            
        # select vehicle position and orientation
        spawn_points = self.world.get_map().get_spawn_points()
        transform = spawn_points[0]
        self.actor.set_transform(transform)
    


    def destroy(self):
        # check the actor exists
        if self.actor is None:
                print("The vehicle object does not exist")
                return

        # destroy and clear
        self.actor.destroy()
        self.actor = None
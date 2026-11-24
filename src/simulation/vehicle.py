
# This file is for spawning and controlling the vehicles

import carla
import math
import random 
import numpy as np

FF_SPEED    = [0.0, 2.8,  5.6,  8.3,   11.1, 13.9, 16.7,  18.1]    # m/s
FF_THROTTLE = [0.0, 0.16, 0.34, 0.385, 0.44, 0.49, 0.555, 0.60]    # 65 km/h entry is a guess
KP = 0.3                    # m/s error -> throttle
KI = 0.1
INTEGRAL_LIMIT = 2.0        # integral can only trim +/-0.2 throttle (KI * 2.0)
DT = 0.05
MAX_UP = 0.03               # normal throttle rise per tick (stops the tick-to-tick flip-flop)
MAX_UP_BIG_ERROR = 0.08     # faster rise when well below target (gear-shift dips)
BIG_ERROR_MS = 0.5          # "well below target" threshold, m/s (~1.8 km/h)
MAX_DOWN = 0.10             # throttle can drop faster than it rises, which cuts overshoot

class Vehicle:

    def __init__(self, world): # constructor
        self.world = world
        self.actor = None
        self.spawn_transform = None
        self.target_speed = None # in m/s
        self.speed_integral = 0.0

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


    def set_target_speed(self, target_speed):
        self.target_speed = target_speed / 3.6

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

        return speed

    def maintain_heading(self):
        target_yaw = self.spawn_transform.rotation.yaw
        current_yaw = self.actor.get_transform().rotation.yaw

        yaw_error = target_yaw - current_yaw

        # Normalize angle between -180 and 180 degrees
        yaw_error = (yaw_error + 180) % 360 - 180

        kp_yaw = 0.02

        steer = kp_yaw * yaw_error

        # Limit steering correction
        steer = max(-0.1, min(steer, 0.1))

        return steer


    def maintain_speed(self):
        # self.ticks_since_start = getattr(self, "ticks_since_start", 0) + 1
        # force_gear = self.ticks_since_start <= 3
        velocity = self.actor.get_velocity()
        current_speed = (velocity.x**2 + velocity.y**2 + velocity.z**2) ** 0.5   # no print spam

        error = self.target_speed - current_speed

        feedforward = float(np.interp(self.target_speed, FF_SPEED, FF_THROTTLE))
        raw = feedforward + KP * error + KI * self.speed_integral
        raw = max(0.0, min(raw, 1.0))

        # first call: start from the feedforward value, not from 0
        last = getattr(self, "last_throttle", None)
        if last is None:
            last = feedforward

        # asymmetric slew limit
        max_up = MAX_UP_BIG_ERROR if error > BIG_ERROR_MS else MAX_UP
        delta = max(-MAX_DOWN, min(raw - last, max_up))
        throttle = max(0.0, min(last + delta, 1.0))
        self.last_throttle = throttle

        # conditional integration (anti-windup): only accumulate while the slew limit is not
        # holding the throttle back, otherwise the integral winds up and causes the overshoot
        if abs(throttle - raw) < 1e-6:
            self.speed_integral += error * DT
            self.speed_integral = max(-INTEGRAL_LIMIT, min(self.speed_integral, INTEGRAL_LIMIT))

        steer = self.maintain_heading()

        control = carla.VehicleControl(
            throttle=throttle,
            steer=steer,
            brake=0.0,
            # manual_gear_shift=force_gear,
            # gear=1 if force_gear else 0
        )

        self.apply_control(control)

      
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
       
        self.spawn_transform = None
        
        self.target_speed = None

import carla
from simulation.vehicle import Vehicle

class AEB:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.state = "NORMAL"
        self.kp = 0.1
        self.clear_frames = 0
        self.required_clear_frames = 10
        self.max_deceleration = 8
        self.brake = 0
        self.last_required_deceleration = 0

    def update(self, risk, ped_in_path, vehicle_speed, distance_between):
        if distance_between is not None:
            self.last_required_deceleration = (
                self.calculate_required_deceleration(
                    vehicle_speed, distance_between
                )
            )

        if self.state == "STOPPED":
            if ped_in_path:
                self.clear_frames = 0
                self.emergency_brake(self.last_required_deceleration)
            else:
                self.clear_frames += 1
                if self.clear_frames >= self.required_clear_frames:
                   
                    self.state = "NORMAL"
                    self.clear_frames = 0
                else: 
                    self.emergency_brake(self.last_required_deceleration)
                    
        elif self.state == "EMERGENCY_BRAKE":
            self.emergency_brake(self.last_required_deceleration)

        elif risk == "NONE":
            self.state = "NORMAL"

        elif risk == "WARNING":
            self.state = "WARNING"

        elif risk == "CRITICAL":
            self.state = "EMERGENCY_BRAKE"
            self.emergency_brake(self.last_required_deceleration)

    def calculate_required_deceleration(self, vehicle_speed, distance_between):
        if distance_between is not None:
            delay = 0.2
            safety_margin = 2.0

            available_distance = (
                distance_between
                - vehicle_speed * delay
                - safety_margin
            )

            if available_distance <= 0:
                return self.max_deceleration

            required_deceleration = (
                vehicle_speed ** 2 / (2 * available_distance)
            )

            return min(required_deceleration, self.max_deceleration)
    
    def emergency_brake(self, required_deceleration):

        # get current longitudinal acceleration
        acceleration = self.vehicle.actor.get_acceleration()

        forward = self.vehicle.actor.get_transform().get_forward_vector()

        longitudinal_acceleration = (
            acceleration.x * forward.x +
            acceleration.y * forward.y +
            acceleration.z * forward.z
        )

        actual_deceleration = -longitudinal_acceleration
        error = required_deceleration - actual_deceleration

        self.brake = self.kp * error
        self.brake = max(0.0, min(self.brake, 1.0))

        control = carla.VehicleControl(
            throttle=0,
            steer=0,
            brake=self.brake
        )
        self.vehicle.apply_control(control)

        if self.vehicle.get_speed() < 0.1:
            self.state = "STOPPED"
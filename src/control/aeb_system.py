
import carla
from simulation.vehicle import Vehicle

class AEB:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.state = "NORMAL"
        self.clear_frames = 0
        self.required_clear_frames = 10

    def update(self, risk, ped_in_path):

        if self.state == "STOPPED":
            if ped_in_path:
                if self.state == "STOPPED":
                    print(f"in AEB: RISK: {risk}, STOPPED: ped_in_path={ped_in_path}, clear_frames={self.clear_frames}")
                    self.clear_frames = 0
                    self.emergency_brake()
            else:
                self.clear_frames += 1
                if self.clear_frames >= self.required_clear_frames:
                    self.state = "NORMAL"
                    self.clear_frames = 0
                else: 
                    self.emergency_brake()
                    
        elif self.state == "EMERGENCY_BRAKE":
            self.emergency_brake()

        elif risk == "NONE":
            print("No Risk")
            self.state = "NORMAL"

        elif risk == "WARNING":
            print("Risk is warning")
            self.state = "WARNING"

        elif risk == "CRITICAL":
            print("Risk is critical , apply breaks")
            self.state = "EMERGENCY_BRAKE"
            self.emergency_brake()

    def emergency_brake(self):

        control = carla.VehicleControl(
            throttle=0,
            steer=0,
            brake=1
        )
        self.vehicle.apply_control(control)

        if self.vehicle.get_speed() < 0.1:
            self.state = "STOPPED"
import carla

class RiskAssessment():
    def __init__(self, reaction_time=0.2, safety_margin = 3):
        self.TTC_BRAKE = 1.5 # seconds left for the AEB to emergency brake
        self.last_ttc = None
        self.last_risk_state = None


    def assess_risk(self, confidence, vehicle_speed, pedestrian_speed, distance_between):

        # estimate ttc, TTC = distance between ped & car / speed - closing speed

        closing_speed = vehicle_speed - pedestrian_speed
        if closing_speed <= 0.001:
            self.last_ttc = float("inf")
        else:
            self.last_ttc = distance_between / closing_speed

        # critical risk
        if self.last_ttc <= self.TTC_BRAKE:
            self.last_risk_state =  "CRITICAL"

        elif self.last_ttc > self.TTC_BRAKE and self.last_ttc <= self.TTC_BRAKE + 1.5:
            self.last_risk_state = "WARNING"

        elif self.last_ttc > self.TTC_BRAKE + 1.5:
            self.last_risk_state = "SAFE"

        return self.last_risk_state


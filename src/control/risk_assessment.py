import carla

class RiskAssessment():
    def __init__(self, reaction_time=0.2, safety_margin = 3):
        self.TTC_BRAKE = 1.5 # seconds left for the AEB to emergency brake
        self.last_ttc = None
        self.last_risk_state = None


    def assess_risk(self, confidence, vehicle_speed, pedestrian_speed, distance_between):

        # is yolo confident enough
        if confidence < .50:
            return "NONE"
       

        # estimate ttc, TTC = distance between ped & car / speed - closing speed

        closing_speed = vehicle_speed - pedestrian_speed
        if closing_speed <= 0.001:
            ttc = float("inf")
        else:
            ttc = distance_between / closing_speed

        # critical risk
        if ttc <= self.TTC_BRAKE:
            self.last_risk_state
            self.last_risk_state =  "CRITICAL"

        elif ttc > self.TTC_BRAKE and ttc <= self.TTC_BRAKE + 1.5:
            self.last_risk_state = "WARNING"

        elif ttc > self.TTC_BRAKE + 1.5:
            self.last_risk_state = "SAFE"

        return self.last_risk_state


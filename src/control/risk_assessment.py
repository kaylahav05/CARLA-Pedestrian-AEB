import carla

class RiskAssessment():
    def __init__(self, reaction_time=0.2, safety_margin = 3):
        self.MAX_DECELERATION = 7
        self.REACTION_TIME = 0.2
        self.TTC_WARNING = 3.0
        self.TTC_CRITICAL = 1.5
        self.CONFIRM_FRAMES = 3
        self.DECEL_WARNING = 3.0

    def assess_risk(self, speed, ped_distance, confidence, ped_speed=0):

        # is yolo confident enough
        if confidence < .50:
            return "NONE"

        closing_speed = speed - ped_speed

        if ped_distance <= 0 or closing_speed <= 0.5:
            return "NONE"

        ttc = ped_distance / closing_speed
        effective_distance = ped_distance - closing_speed * self.REACTION_TIME
        if effective_distance <= 0:
            return "CRITICAL"
        
        req_deceleration = closing_speed ** 2 / (2 *effective_distance)

        if ttc < self.TTC_CRITICAL or req_deceleration > 0.7 * self.MAX_DECELERATION:
            print (f"TTC: {ttc}")
            return "CRITICAL"
        if ttc < self.TTC_WARNING or req_deceleration > self.DECEL_WARNING:
            return "WARNING"
        return "NONE"

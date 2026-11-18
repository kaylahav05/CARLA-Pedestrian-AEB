
# this file hold all the scenario configurations
import carla

SCENARIOS = {

    "basic_crossing": {

        # Vehicle settings
        "vehicle_id": "vehicle.tesla.model3",
        "vehicle_spawn": carla.Transform(
            carla.Location(x=-28.58, y=140.54, z=0.60),
            carla.Rotation(yaw=0.35)
            ),
        "vehicle_destination": carla.Location(
            x=35.0, y=140.54, z=0.60
        ),
        "vehicle_target_speed": 30.0,

        # Pedestrian settings
        "pedestrian_id": "walker.pedestrian.0007",
        "pedestrian_spawn": carla.Transform(
            carla.Location(x=6.42, y=135.54, z=0.60),
            carla.Rotation(yaw=90.0)
        ),
        "pedestrian_destination": carla.Location(
            x=6.42, y=146.54, z=0.60
        ),
        "pedestrian_speed": 2.5,

        # Crossing trigger
        "crossing_trigger_distance": 20.0,
        # Scenario duration
        "max_duration": 30.0
    }
}
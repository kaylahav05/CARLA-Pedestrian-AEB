

FIXED = {
    "town": "Town04",
    "crossing_point": {"x": -229.42, "y": 432.18, "z": 0.20},
    "yaw" : -0.21,
    "vehicle_id": "vehicle.tesla.model3",
    "vehicle_front_offset_m": 2.425,
    "initial_ttc": 4.0,
    "max_duration": 15.0,
    "vehicle_width" : 2.163450002670288
}

SCENARIOS = {
    "crossing-road": {
        "variations": {
            "unobstructed-right": {
                "section": "S8.3.1",
                "direction": "right-left",
                "pedestrian_id": "walker.pedestrian.0007",
                "pedestrian_speed": 5.0,
                "pedestrian_start_offset": 4.0, # meter
                "obstructed": False,
                "vehicle_speeds": [10, 20, 30, 40, 50, 60],
                "conditions": [
                    {"lighting": "day", "overlap": 0.25},
                    {"lighting": "day", "overlap": 0.50},
                    {"lighting": "low-beam", "overlap": 0.50},
                    {"lighting": "high-beam", "overlap": 0.50}
                ]
            },

            "unobstructed-left": {
                "section": "S8.3.2",
                "direction": "left-right",
                "pedestrian_id": "walker.pedestrian.0007",
                "pedestrian_speed": 8.0,
                "pedestrian_start_offset": 6.0,
                "obstructed": False,
                "vehicle_speeds": [10, 20, 30, 40, 50, 60],
                "conditions": [
                    {"lighting": "day", "overlap": 0.50}
                ]
            },

            "obstructed-right": {
                "section": "S8.3.3",
                "direction": "right-left",
                "pedestrian_id": "walker.pedestrian.0014",
                "pedestrian_speed": 5.0,
                "pedestrian_start_offset": 4.0,
                "obstructed": True,
                "obstruction_blueprint": "vehicle.tesla.model3",
                "obstruction_length": 4.7,
                "obstruction_width": 1.9,
                "vehicle_speeds": [10, 20, 30, 40, 50],
                "conditions": [
                    {"lighting": "day", "overlap": 0.50}
                ]
            }
        }
    },
    "stationary" : {
        "variations" : {
            "stationary" : {
                "section": "S8.4",
                "direction": "right-left",
                "pedestrian_id": "walker.pedestrian.0007",
                "pedestrian_speed": 0,
                "obstructed": False,
                "vehicle_speeds": [10, 20, 30, 40, 50, 55],
                "conditions": [
                    {"lighting": "day", "overlap": 0.25},
                    {"lighting": "low-beam", "overlap": 0.25},
                    {"lighting": "high-beam", "overlap": 0.25}
                ]
            },

        }
    },
    "along-path" : {
        "variations" : {
            "along-path" : {
                "section": "S8.5",
                "direction": "right-forward",
                "pedestrian_id": "walker.pedestrian.0007",
                "pedestrian_speed": 5,
                "obstructed": False,
                "vehicle_speeds": [10, 20, 30, 40, 50, 60, 65],
                "conditions": [
                    {"lighting": "day", "overlap": 0.25},
                    {"lighting": "low-beam", "overlap": 0.25},
                    {"lighting": "high-beam", "overlap": 0.25}
                ]
            },
        
        }

    }
}
import carla

def crossing_config(v_kmh, ped_speed=2.5, side="left", d_lat=5.0,
                    offset=0.0, lead=0.0,
                    x_c=6.42, y_lane=140.54, front_offset=2.43):
    """
    d_lat:  start distance from lane center (m)
    offset: aim point across the lane (+ = far side)
    lead:   seconds the pedestrian reaches the aim point BEFORE the bumper
            (0 = collision course, +1.5 = clears early, -1.5 = arrives late)
    """
    v = v_kmh / 3.6
    walk_time = (d_lat + offset) / ped_speed if ped_speed > 0 else 0.0
    trigger_dist = v * (walk_time + lead)

    s = 1 if side == "left" else -1         # +y is to the car's right in CARLA
    x_spawn = x_c - front_offset - trigger_dist - 5.0   # 5 m of margin

    return {
        "vehicle_id": "vehicle.tesla.model3",
        "vehicle_spawn": carla.Transform(
            carla.Location(x=x_spawn, y=y_lane, z=0.6), carla.Rotation(yaw=0.35)),
        "vehicle_destination": carla.Location(x=31.0, y=141, z=0),
        "vehicle_target_speed": v_kmh,

        "pedestrian_id": "walker.pedestrian.0007",
        "pedestrian_spawn": carla.Transform(
            carla.Location(x=x_c, y=y_lane - s * d_lat, z=0.6),
            carla.Rotation(yaw=90.0 * s)),
        "pedestrian_destination": carla.Location(
            x=x_c, y=y_lane + s * (d_lat + 6.0), z=0.6),
        "pedestrian_speed": ped_speed,

        "crossing_point": carla.Location(x=x_c, y=y_lane, z=0),
        "crossing_trigger_distance": trigger_dist,
        "max_duration": 30.0,
    }
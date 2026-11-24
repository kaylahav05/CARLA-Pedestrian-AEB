import csv
from pathlib import Path
from datetime import datetime


class CSVLogger:
    def __init__(self):
        # Create logs directory in project root
        project_root = Path(__file__).resolve().parents[2]
        log_dir = project_root / "logs"
        log_dir.mkdir(parents=True, exist_ok=True)

        # One CSV per execution of the full test suite
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

        filename = f"aeb_tests_{timestamp}.csv"
        self.filepath = log_dir / filename

        # Columns to record
        self.fields = [
            "test_id",
            "run_number",
            "tick",
            "simulation_time_s",

            # Vehicle
            "vehicle_speed_ms",
            "vehicle_acceleration_ms2",
            "vehicle_brake",
            "vehicle_throttle",
            "vehicle_steer",
            "vehicle_yaw_deg",
            "vehicle_yaw_rate_deg_s",
            "front_left_steer_deg",
            "front_right_steer_deg",  
            "vehicle_gear", 

            # Pedestrian
            "ped_speed_ms",
            "ped_distance_to_crossing_m",
            "ped_crossing_started",

            # Distances
            "vehicle_ped_distance_m",
            "vehicle_front_to_crossing_m",
            "ped_longitudinal_distance_m",
            "ped_lateral_distance_m",

            # Scenario
            "target_overlap_pct",
            "vehicle_target_speed_kmh",
            "ped_target_speed_kmh",

            # Detection
            "ped_detected",
            "detection_confidence",
            "ped_in_path",

            # Risk assessment
            "ttc_s",
            "ttc_brake_s",
            "risk_state",

            # AEB
            "aeb_state",
            "required_deceleration_ms2",
            "brake_command",

            # Results
            "collision_occurred"

        
        ]

        self.file = open(self.filepath, "w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(
            self.file,
            fieldnames=self.fields
        )

        self.writer.writeheader()

    def log_tick(self, data):
        rounded_data = {}

        for field in self.fields:
            value = data.get(field)

            if isinstance(value, float):
                value = round(value, 3)

            rounded_data[field] = value

        self.writer.writerow(rounded_data)

    def end_run(self):
        self.writer.writerow({})
        self.file.flush()

    def close(self):
        if not self.file.closed:
            self.file.close()
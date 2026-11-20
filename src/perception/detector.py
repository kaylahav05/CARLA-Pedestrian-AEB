
import carla
import math
from ultralytics import YOLO
import numpy as np
import cv2
from simulation.pedestrian import Pedestrian
from simulation.vehicle import Vehicle

class Detector:
    def __init__(self, world, pedestrian, vehicle):
        self.world = world
        self.pedestrian = pedestrian
        self.vehicle = vehicle
        
        self.model = YOLO("yolo11n.pt")
        self.confidence_threshold = .5 # temp min condidence

        self.path_polygon = np.array([
            [375, 300],  # Top left
            [425, 300],  # Top right
            [713, 590],  # Bottom right
            [87, 590]   # Bottom left
        ], dtype=np.int32)

    def detect_pedestrians(self, frame):
        if frame is None:
            return []

        # run yolo on the frame to get detected objs
        results = self.model(frame, verbose=False)
        detections = []

        # process each object yolo detects in frame
        for box in results[0].boxes:
            # 0 represents a person
            class_id = int(box.cls[0]) 
            confidence = float(box.conf[0])

            # only accept pedestrians above confidence threshold
            if class_id == 0 and confidence >= self.confidence_threshold:
                # get bounding boxes
                x1, y1, x2, y2 = box.xyxy[0].tolist()

                detections.append({
                    "confidence": confidence,
                    "bounding_box": [x1, y1, x2, y2]
                })
        return detections


    def get_pedestrian_distance(self):
        vehicle_transform = self.vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        yaw = math.radians(vehicle_transform.rotation.yaw)

        ped_location = self.pedestrian.get_location()

        # forward direction of the car
        fwd_x, fwd_y = math.cos(yaw), math.sin(yaw)

        # vector from the car to the pedestrian
        dx = ped_location.x - vehicle_location.x
        dy = ped_location.y - vehicle_location.y

        # distance ahead of the car's origin, ignoring sideways offset
        along = dx * fwd_x + dy * fwd_y

        # shift the reference point from the car's origin to the front bumper
        bbox = self.vehicle.bounding_box
        front_offset = bbox.location.x + bbox.extent.x

        return along - front_offset


    def is_in_path(self, detection):

        # get the bounding box
        x1, y1, x2, y2 = detection["bounding_box"]

        # get pedestrian's feet
        foot_x = (x1 + x2) / 2
        foot_y = y2

        # check if feet are in the trapezoid
        result = cv2.pointPolygonTest(
            self.path_polygon,
            (float(foot_x), float(foot_y)),
            False
        )

        if result >= 0:
            return True
        elif result == -1:
            return False

    def is_near_path(self, detection, margin=50):
            x1, y1, x2, y2 = detection["bounding_box"]

            foot_x = (x1 + x2) / 2
            foot_y = y2

            distance = cv2.pointPolygonTest(
                self.path_polygon,
                (float(foot_x), float(foot_y)),
                True
            )

            return distance >= -margin
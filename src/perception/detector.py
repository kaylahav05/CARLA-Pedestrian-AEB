
import carla
import math
from ultralytics import YOLO


class Detector:
    def __init__(self, world, pedestrian, vehicle):
        self.world = world
        self.vehicle = vehicle
        self.pedestrian = pedestrian
        
        self.model = YOLO("yolo11n.pt")
        self.confidence_threshold = .5 # temp min condidence

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
        vehicle_location = self.vehicle.get_location()
        pedestrian_location = self.pedestrian.get_location()

        distance = vehicle_location.distance(pedestrian_location)

        return distance
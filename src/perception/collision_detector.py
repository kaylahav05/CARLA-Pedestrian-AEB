import math
import threading

import carla
import numpy as np


class CollisionDetector:
    def __init__(self, world, ego_actor, pedestrian_actor, post_collision_duration=3.0):
        self.world = world
        self.ego = ego_actor
        self.ped = pedestrian_actor
        self.post_collision_duration = post_collision_duration

        self._sensor = None
        self._lock = threading.Lock()
        self._ped_id = pedestrian_actor.id 
        self.detected = False
        self.source = None      # "sensor" or "geometric"
        self.time = None        # sim seconds
        self.frame = None
        self.speed = None       # ego speed (m/s) when first noticed

    # setup / teardown
    def attach(self):
        bp = self.world.get_blueprint_library().find("sensor.other.collision")
        self._sensor = self.world.spawn_actor(
            bp, carla.Transform(), attach_to=self.ego
        )
        self._sensor.listen(self._on_sensor_event)

    def destroy(self):
        # stops the sensor but keeps the recorded results readable
        if self._sensor is not None:
            self._sensor.stop()
            self._sensor.destroy()
            self._sensor = None

    # detection
    def _register(self, source, time_s, frame):
        with self._lock:
            if self.detected:
                return
            self.detected = True
            self.source = source
            self.time = time_s
            self.frame = frame

    def _on_sensor_event(self, event):      # sensor thread
        if event.other_actor.id == self._ped_id:
            self._register("sensor", event.timestamp, event.frame)

    def update(self):
        """Call once per tick from Scenario.update()."""
        if not self.detected and self._boxes_overlap():
            snap = self.world.get_snapshot()
            self._register("geometric", snap.timestamp.elapsed_seconds, snap.frame)

        if self.detected and self.speed is None:
            v = self.ego.get_velocity()
            self.speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)

    def post_collision_complete(self):
        if self.time is None:
            return False
        now = self.world.get_snapshot().timestamp.elapsed_seconds
        return now - self.time >= self.post_collision_duration

    # geometry (separating axis test on 2D oriented boxes)
    @staticmethod
    def _corners(actor):
        verts = actor.bounding_box.get_world_vertices(actor.get_transform())
        return np.array([(v.x, v.y) for v in verts])

    def _boxes_overlap(self):
        a, b = self._corners(self.ego), self._corners(self.ped)
        axes = []
        for actor in (self.ego, self.ped):
            yaw = math.radians(
                actor.get_transform().rotation.yaw + actor.bounding_box.rotation.yaw
            )
            axes.append(np.array([math.cos(yaw), math.sin(yaw)]))
            axes.append(np.array([-math.sin(yaw), math.cos(yaw)]))

        for axis in axes:
            pa, pb = a @ axis, b @ axis
            if pa.max() < pb.min() or pb.max() < pa.min():
                return False
        return True
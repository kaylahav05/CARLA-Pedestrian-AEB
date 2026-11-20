
# This file is for attaching the RGB camera to the vehicle and displaying the feed
import carla
import numpy as np
import cv2

class Camera:
    def __init__(self, world, vehicle): # constructor
            self.world = world
            self.vehicle = vehicle
            self.camera = None
            self.frame = None

    def attach_camera(self):

        # get the camera from the blueprint library and adjust it
        blueprint_library = self.world.get_blueprint_library()
        camera_blueprint = blueprint_library.find("sensor.camera.rgb")

        camera_blueprint.set_attribute("image_size_x", "800")
        camera_blueprint.set_attribute("image_size_y", "600")
        camera_blueprint.set_attribute("fov", "90")

        camera_transform = carla.Transform(carla.Location(x=0.8, z=1.7))

        # spawn the camera and get its images
        self.camera = self.world.spawn_actor(camera_blueprint, camera_transform, attach_to=self.vehicle)
        self.camera.listen(self.process_image)

    def process_image(self, image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        self.frame = array[:, :, :3].copy()

    def display(self, path_polygon=None):
        """
        # show the images in the window
        if self.frame is not None:
            cv2.imshow("CARLA RGB Camera", self.frame)
            cv2.waitKey(1)

        """
        if self.frame is None:
            return

        # Copy the image so we don't modify the original frame
        display_frame = self.frame.copy()

        # Draw driving path
        if path_polygon is not None:
            cv2.polylines(
                display_frame,
                [path_polygon],
                isClosed=True,
                color=(0, 255, 0),
                thickness=2
            )

        cv2.imshow("CARLA RGB Camera", display_frame)
        cv2.waitKey(1)

    def destroy(self):
        if self.camera is not None:
            self.camera.stop()
            self.camera.destroy()
            self.camera = None

        cv2.destroyAllWindows()
         
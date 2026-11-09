import cv2
import numpy as np

from .stereo import Calibration


def calibration():
    return Calibration(np.array([[360., 0., 320.], [0., 360., 240.], [0., 0., 1.]]),
                       0.25, np.eye(4))


class Scene:
    """Synthetic textured landmarks; not a physical photometric camera model."""
    def __init__(self, seed=17):
        random = np.random.default_rng(seed)
        self.landmarks = random.uniform([-4, -3, 5], [4, 3, 12], (350, 3))
        self.patches = random.integers(30, 240, (350, 11, 11), dtype=np.uint8)
        self.calibration = calibration()

    def image(self, position):
        camera_points = self.landmarks - position
        projected = camera_points @ self.calibration.intrinsic.T
        pixels = np.rint(projected[:, :2] / projected[:, 2:]).astype(int)
        frame = np.full((480, 640), 15, dtype=np.uint8)
        for index in np.argsort(camera_points[:, 2])[::-1]:
            u, v = pixels[index]
            if 6 <= u < 634 and 6 <= v < 474 and camera_points[index, 2] > 0:
                frame[v-5:v+6, u-5:u+6] = self.patches[index]
        return cv2.GaussianBlur(frame, (3, 3), 0.4)

    def stereo(self, position):
        return self.image(position), self.image(position + [self.calibration.baseline_m, 0, 0])

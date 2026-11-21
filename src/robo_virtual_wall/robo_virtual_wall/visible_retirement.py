"""Retire missing tape only from a current view while the robot is stationary."""
import math
import cv2
import numpy as np
from .projection import to_ground


def observed_ground(frame, calibration, roi_top, roi_bottom, max_range):
    """Current calibrated image support; dark/featureless frames give no evidence.

    Image appearance does not certify unobstructed ground. In particular, this
    cannot distinguish an unseen tape from tape covered by a bright object.
    """
    matrix, size, polygon = calibration
    if frame.shape[1::-1] != size:
        return None
    width, height = size
    rectangle = np.float32([[0, int(height*roi_top)], [width-1, int(height*roi_top)],
                           [width-1, int(height*roi_bottom)-1], [0, int(height*roi_bottom)-1]])
    area, intersection = cv2.intersectConvexConvex(polygon.astype(np.float32), rectangle)
    if intersection is None or area < 100:
        return None
    pixels = intersection.reshape(-1, 2)
    mask = np.zeros((height, width), np.uint8)
    cv2.fillPoly(mask, [np.rint(pixels).astype(np.int32)], 255)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    values = gray[mask != 0]
    if values.mean() < 35 or values.std() < 8:
        return None
    homogeneous = to_ground(calibration, pixels)
    denominator = homogeneous[:, 2]
    if (np.abs(denominator) < 1e-9).any() or denominator.min()*denominator.max() <= 0:
        return None
    points = homogeneous[:, :2] / denominator[:, None]
    if not np.isfinite(points).all() or (points[:, 0] <= 0).any() or (np.linalg.norm(points, axis=1) > max_range).any():
        return None
    return points


class VisibleRetirement:
    def __init__(self, timeout=2., min_frames=6, max_gap=.5, edge_tolerance=.01):
        if not math.isfinite(timeout) or timeout <= 0 or min_frames < 2 or max_gap <= 0:
            raise ValueError('Invalid tape retirement settings')
        self.timeout, self.min_frames = timeout, min_frames
        self.max_gap, self.edge_tolerance = max_gap, edge_tolerance
        self.absent = {}
        self.last_stamp = None

    def reset(self):
        self.absent.clear()
        self.last_stamp = None

    @staticmethod
    def supported(wall, observation):
        axis = wall[1]-wall[0]
        length = np.linalg.norm(axis)
        other = observation[1]-observation[0]
        other_length = np.linalg.norm(other)
        if min(length, other_length) < 1e-9:
            return False
        axis /= length
        if abs(axis @ other/other_length) < math.cos(math.radians(10)):
            return False
        offset = observation-wall[0]
        distance = np.abs(offset @ np.array([-axis[1], axis[0]]))
        along = offset @ axis
        overlap = max(0., min(length, along.max())-max(0., along.min()))
        return distance.max() <= .08 and overlap >= min(.08, length*.25)

    def update(self, walls, observations, view, stamp, stationary):
        if not math.isfinite(stamp):
            self.reset()
            return 0
        if self.last_stamp is not None:
            if stamp <= self.last_stamp:
                return 0
            if stamp-self.last_stamp > self.max_gap:
                self.absent.clear()
        self.last_stamp = stamp
        if not stationary or view is None:
            self.absent.clear()
            return 0
        polygon = np.asarray(view, dtype=np.float32)
        if (polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3
                or not np.isfinite(polygon).all() or not cv2.isContourConvex(polygon)):
            self.absent.clear()
            return 0
        retained = []
        absent = {}
        for wall in walls:
            key = id(wall)
            visible = all(cv2.pointPolygonTest(polygon, tuple(map(float, p)), True)
                          >= -self.edge_tolerance for p in wall)
            observed = any(self.supported(wall, item) for item in observations)
            if visible and not observed:
                first, count = self.absent.get(key, (stamp, 0))
                count += 1
                if stamp-first >= self.timeout and count >= self.min_frames:
                    continue
                absent[key] = (first, count)
            retained.append(wall)
        removed = len(walls)-len(retained)
        walls[:] = retained
        self.absent = absent
        return removed

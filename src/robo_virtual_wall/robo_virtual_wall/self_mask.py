"""Measured-image exclusion of the robot itself, for a fixed camera/arm pose."""
import json
import cv2
import numpy as np


class SelfMask:
    def __init__(self, path):
        data = json.loads(path.read_text())
        self.size = tuple(data['image_size'])
        if len(self.size) != 2 or any(type(v) is not int or v <= 0 for v in self.size):
            raise ValueError('Invalid self-mask image size')
        self.mask = np.zeros(self.size[::-1], np.uint8)
        self.polygons = []
        for vertices in data['polygons']:
            p = np.asarray(vertices, dtype=float)
            if p.ndim != 2 or p.shape[1] != 2 or len(p) < 3 or not np.isfinite(p).all():
                raise ValueError('Invalid self-mask polygon')
            if (p < 0).any() or (p >= self.size).any():
                raise ValueError('Self-mask polygon outside reference image')
            p = np.rint(p).astype(np.int32)
            self.polygons.append(p)
            cv2.fillPoly(self.mask, [p], 255)
        if not self.polygons:
            raise ValueError('Self-mask needs at least one polygon')

    def check_size(self, image):
        if image.shape[1::-1] != self.size:
            raise ValueError('Camera resolution changed: self-mask must be remeasured')

    def erase(self, image):
        self.check_size(image)
        result = image.copy()
        result[self.mask != 0] = 0
        return result

    def visible_segments(self, segments):
        """Split axes crossing the robot, including gaps rejoined by morphology."""
        result = []
        for a, b, angle in segments:
            a, b = np.asarray(a, float), np.asarray(b, float)
            count = max(2, int(np.ceil(np.linalg.norm(b-a))) + 1)
            points = np.rint(np.linspace(a, b, count)).astype(int)
            inside = ((points >= 0) & (points < self.size)).all(axis=1)
            visible = np.zeros(count, bool)
            visible[inside] = self.mask[points[inside, 1], points[inside, 0]] == 0
            edges = np.flatnonzero(np.diff(np.r_[False, visible, False]))
            for start, end in zip(edges[::2], edges[1::2]):
                if np.linalg.norm(points[end-1]-points[start]) >= 8:
                    result.append((tuple(points[start]), tuple(points[end-1]), angle))
        return result

    def draw(self, image):
        self.check_size(image)
        cv2.polylines(image, self.polygons, True, (255, 0, 255), 1)

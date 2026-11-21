"""Image-only articulated gripper tracking. Loss of any part invalidates vision.

Three local rigid models approximate the articulated arm, rather than a single
translated silhouette. SIFT appearance anchors acquisition; forward/backward LK
tracks each part between frames. This is not semantic object recognition.
"""
import json
import time
from pathlib import Path
import cv2
import numpy as np
from .self_mask import SelfMask


class DynamicSelfMask(SelfMask):
    def __init__(self, path):
        data = json.loads(Path(path).read_text())
        self.size = tuple(data['image_size'])
        reference = Path(path).parent / data['reference_image']
        self.reference = cv2.imread(str(reference))
        if self.reference is None or self.reference.shape[1::-1] != self.size:
            raise ValueError('Dynamic gripper reference image missing or wrong size')
        self.sift = cv2.SIFT_create(nfeatures=2000, contrastThreshold=.02)
        self.reference_gray = cv2.cvtColor(self.reference, cv2.COLOR_BGR2GRAY)
        self.minimum = 6
        self.parts = []
        for item in data['parts']:
            poly = np.asarray(item['polygon'], np.float32)
            if (poly.ndim != 2 or poly.shape[1] != 2 or len(poly) < 3
                    or not np.isfinite(poly).all() or (poly < 0).any()
                    or (poly >= self.size).any()):
                raise ValueError('Invalid dynamic part polygon')
            support = np.zeros(self.size[::-1], np.uint8)
            feature_polygons = item.get('feature_polygons', [item['polygon']])
            for vertices in feature_polygons:
                feature_poly = np.asarray(vertices, dtype=float)
                if (feature_poly.ndim != 2 or feature_poly.shape[1] != 2
                        or len(feature_poly) < 3 or not np.isfinite(feature_poly).all()
                        or (feature_poly < 0).any() or (feature_poly >= self.size).any()):
                    raise ValueError('Invalid feature support polygon')
                cv2.fillPoly(support, [np.rint(feature_poly).astype(np.int32)], 255)
            silhouette = np.zeros_like(support)
            cv2.fillPoly(silhouette, [np.rint(poly).astype(np.int32)], 255)
            support = cv2.bitwise_and(support, silhouette)
            support = cv2.erode(support, np.ones((5, 5), np.uint8))
            keys, descriptors = self.sift.detectAndCompute(self.reference_gray, support)
            if descriptors is None or len(keys) < self.minimum:
                raise ValueError('Insufficient visual features in gripper part: '+item['name'])
            self.parts.append(dict(name=item['name'], polygon=poly,
                reference_keys=np.array([k.pt for k in keys], np.float32),
                descriptors=descriptors, transform=None, anchors=None,
                points=None, inliers=0))
        if len(self.parts) < 2:
            raise ValueError('Dynamic mask requires separate articulated parts')
        self.mask = np.zeros(self.size[::-1], np.uint8)
        self.polygons = []
        self.previous_gray = None
        self.previous_time = None
        self.frames = 0
        self.valid = False
        self.reason = 'aguardando reconhecimento da garra'

    @property
    def status(self):
        return dict(mode='dynamic', valid=self.valid, reason=self.reason,
                    parts=[dict(name=p['name'], inliers=p['inliers'],
                                tracked=p['transform'] is not None) for p in self.parts])

    def _fit(self, part, anchors, points):
        if len(points) < self.minimum:
            return None
        matrix, inliers = cv2.estimateAffinePartial2D(anchors, points,
            method=cv2.RANSAC, ransacReprojThreshold=2., maxIters=1000)
        if matrix is None or not np.isfinite(matrix).all():
            return None
        good = inliers.ravel().astype(bool)
        required = max(self.minimum, int(np.ceil(len(part['reference_keys']) * .25)))
        if good.sum() < required or good.mean() < .60:
            return None
        scale = np.linalg.norm(matrix[:, 0])
        if not .45 <= scale <= 2.0:
            return None
        # Several repeated descriptors at one corner cannot locate a part.
        span = np.ptp(anchors[good], axis=0)
        reference_span = np.ptp(part['polygon'], axis=0)
        if (span < np.maximum(6., reference_span * .35)).any():
            return None
        residual = np.linalg.norm(cv2.transform(anchors[good][None], matrix)[0]-points[good], axis=1)
        if np.median(residual) > 1.3:
            return None
        poly = cv2.transform(part['polygon'][None], matrix)[0]
        if (poly.max(axis=0) < 0).any() or (poly.min(axis=0) >= self.size).any():
            return None
        visible = ((points[good] >= 0) & (points[good] < self.size)).all(axis=1)
        if visible.sum() < self.minimum:
            return None
        return matrix, anchors[good][visible], points[good][visible]

    def _acquire(self, gray, part, keys, descriptors):
        if descriptors is None or len(descriptors) < 2:
            return None
        pairs = cv2.BFMatcher(cv2.NORM_L2).knnMatch(part['descriptors'], descriptors, k=2)
        matches = [a for a, b in pairs if a.distance < .65*b.distance]
        unique = {}
        for match in matches:
            if match.trainIdx not in unique or match.distance < unique[match.trainIdx].distance:
                unique[match.trainIdx] = match
        matches = list(unique.values())
        if len(matches) < self.minimum:
            return None
        anchors = part['reference_keys'][[m.queryIdx for m in matches]]
        points = np.array([keys[m.trainIdx].pt for m in matches], np.float32)
        candidate = self._fit(part, anchors, points)
        if candidate is not None and part['transform'] is not None:
            old = cv2.transform(part['polygon'][None], part['transform'])[0]
            new = cv2.transform(part['polygon'][None], candidate[0])[0]
            if np.median(np.linalg.norm(new-old, axis=1)) > 35:
                return None
        return candidate

    def _flow(self, gray, part):
        old = part['points'].reshape(-1, 1, 2)
        options = dict(winSize=(25, 25), maxLevel=3,
                       criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, .01))
        new, forward, _ = cv2.calcOpticalFlowPyrLK(self.previous_gray, gray, old, None, **options)
        if new is None:
            return None
        back, reverse, _ = cv2.calcOpticalFlowPyrLK(gray, self.previous_gray, new, None, **options)
        if back is None:
            return None
        good = (forward.ravel() != 0) & (reverse.ravel() != 0)
        good &= np.linalg.norm(back[:, 0]-old[:, 0], axis=1) < 1.
        good &= np.linalg.norm(new[:, 0]-old[:, 0], axis=1) < 45.
        return self._fit(part, part['anchors'][good], new[good, 0])

    def update(self, frame, now=None):
        self.check_size(frame)
        now = time.monotonic() if now is None else now
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        continuous = self.previous_time is not None and 0 < now-self.previous_time <= .6
        candidates = []
        keys = descriptors = None
        for part in self.parts:
            candidate = None
            if continuous and part['transform'] is not None:
                candidate = self._flow(gray, part)
            # Re-anchor to appearance regularly; motion alone must not drift forever.
            if candidate is None or self.frames % 8 == 0:
                if keys is None:
                    keys, descriptors = self.sift.detectAndCompute(gray, None)
                anchored = self._acquire(gray, part, keys, descriptors)
                if anchored is not None:
                    candidate = anchored
                elif self.frames % 8 == 0:
                    candidate = None
            candidates.append(candidate)
        self.polygons = []
        self.mask.fill(0)
        for part, candidate in zip(self.parts, candidates):
            if candidate is None:
                part.update(transform=None, anchors=None, points=None, inliers=0)
                continue
            matrix, anchors, points = candidate
            part.update(transform=matrix, anchors=anchors, points=points, inliers=len(points))
            polygon = np.rint(cv2.transform(part['polygon'][None], matrix)[0]).astype(np.int32)
            self.polygons.append(polygon)
            cv2.fillPoly(self.mask, [polygon], 255)
        self.valid = all(c is not None for c in candidates)
        if self.valid:
            centres = np.array([poly.mean(axis=0) for poly in self.polygons])
            body = self.polygons[-1]
            # Fixed-camera physical anchor: the arm body must still reach its base.
            connected = np.max(np.linalg.norm(centres[:, None]-centres[None, :], axis=2)) < 330
            anchored = body[:, 1].max() >= 390 and body[:, 0].min() < 410 and body[:, 0].max() > 110
            self.valid = bool(connected and anchored)
        self.mask = cv2.dilate(self.mask, np.ones((11, 11), np.uint8))
        if cv2.countNonZero(self.mask) > .45 * self.mask.size:
            self.valid = False
        self.reason = 'rastreando partes da garra' if self.valid else 'rastreio insuficiente: deteccoes suspensas'
        self.previous_gray, self.previous_time = gray, now
        self.frames += 1
        return self.valid

    def draw(self, image):
        self.check_size(image)
        color = (255, 0, 255) if self.valid else (0, 0, 255)
        contours, _ = cv2.findContours(self.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(image, contours, -1, color, 1)
        cv2.putText(image, 'GARRA: '+('RASTREANDO' if self.valid else 'SEM RASTREIO - FITAS SUSPENSAS'),
                    (8, 64), cv2.FONT_HERSHEY_SIMPLEX, .43, color, 1)

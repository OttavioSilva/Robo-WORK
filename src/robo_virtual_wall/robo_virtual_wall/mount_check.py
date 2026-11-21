"""CAMERA_MEXEU_20261121: detect a camera that moved since calibration.

Two patches of the robot's own body (bottom-left and bottom-right of the
image, below roi_bottom) are saved with the calibration. At runtime each is
searched in a small window, also over small rotations. Only when BOTH match
well and one of them moved beyond the tolerance (shift or rotation; the two
corners shift differently when the camera rotates) is the mount declared
moved; an occluded patch
(gripper, cable) gives 'inconclusive', never 'moved'. A few consecutive
agreeing checks are required before the state changes.
"""
import cv2
import numpy as np


def choose_patches(gray, roi_bottom=0.85, size=(96, 56)):
    """Most textured window in each bottom corner region (robot body)."""
    height, width = gray.shape
    w, h = size
    # As high as the robot-body band allows, leaving room below for shifts.
    top = min(int(height * roi_bottom), height - h - 4)
    energy = cv2.magnitude(cv2.Sobel(gray, cv2.CV_32F, 1, 0), cv2.Sobel(gray, cv2.CV_32F, 0, 1))
    rects = []
    # Image corners only: the centre holds the gripper and rail, which move.
    for x_range in (range(0, int(width * .25) - w + 1, 4), range(int(width * .75), width - w + 1, 4)):
        best = max(((float(energy[top:top + h, x:x + w].mean()), x) for x in x_range))
        rects.append([best[1], top, w, h])
    return rects


class MountCheck:
    def __init__(self, reference_gray, rects, tolerance_px=3., search_px=24, min_score=.72, confirm=3,
                 angles_deg=(-3., -1.5, 0., 1.5, 3.), angle_tolerance_deg=1., strong_score=.9,
                 angle_margin=.03):
        self.patches = []
        for x, y, w, h in rects:
            patch = reference_gray[y:y + h, x:x + w]
            if patch.shape != (h, w) or float(patch.std()) < 4.:
                raise ValueError('Mount reference patch is outside the image or featureless')
            rotated = []
            for angle in angles_deg:
                # Rotate about the patch centre using the surrounding reference.
                m = cv2.getRotationMatrix2D((x + w / 2, y + h / 2), angle, 1.)
                turned = cv2.warpAffine(reference_gray, m, reference_gray.shape[::-1],
                                        borderMode=cv2.BORDER_REPLICATE)[y:y + h, x:x + w]
                rotated.append((float(angle), turned))
            self.patches.append((rotated, int(x), int(y), w, h))
        self.angle_tolerance = angle_tolerance_deg
        self.strong_score, self.angle_margin = strong_score, angle_margin
        self.tolerance, self.search, self.min_score = tolerance_px, search_px, min_score
        self.confirm = confirm
        self.moved = False
        self.pending = []
        self.last = dict(state='unknown')

    def measure(self, gray):
        """One observation: ('ok'|'moved'|'inconclusive', shift px, scores)."""
        shifts, scores, angles = [], [], []
        for rotated, x, y, w, h in self.patches:
            x0, y0 = max(0, x - self.search), max(0, y - self.search)
            x1, y1 = min(gray.shape[1], x + w + self.search), min(gray.shape[0], y + h + self.search)
            window = gray[y0:y1, x0:x1]
            if window.shape[0] < h or window.shape[1] < w:
                return 'inconclusive', None, []
            candidates = []
            for angle, patch in rotated:
                result = cv2.matchTemplate(window, patch, cv2.TM_CCOEFF_NORMED)
                _, score, _, (bx, by) = cv2.minMaxLoc(result)
                candidates.append((float(score), np.array([bx + x0 - x, by + y0 - y], float), angle))
            straight = next(c for c in candidates if c[2] == 0.)
            best = max(candidates, key=lambda c: c[0])
            # Blur or noise make angles tie: a rotation must clearly win.
            if best[2] != 0. and best[0] < straight[0] + self.angle_margin:
                best = straight
            scores.append(best[0])
            shifts.append(best[1])
            angles.append(best[2])
        worst = max(shifts, key=np.linalg.norm)
        displaced = [np.linalg.norm(s) > self.tolerance or abs(a) >= self.angle_tolerance
                     for s, a in zip(shifts, angles)]
        if min(scores) < self.min_score:
            # One corner may leave the frame when the camera rotates: a single
            # very strong, clearly displaced match is still proof of movement.
            for score, shift, angle in zip(scores, shifts, angles):
                if score >= self.strong_score and (np.linalg.norm(shift) > 2 * self.tolerance
                                                   or abs(angle) >= 1.5):
                    return 'moved', shift, scores
            return 'inconclusive', None, scores
        if not any(displaced):
            return 'ok', worst, scores
        return 'moved', worst, scores

    def update(self, gray):
        """Returns True while the mount is considered moved (debounced)."""
        state, shift, scores = self.measure(gray)
        self.last = dict(state=state, shift_px=None if shift is None else shift.round(1).tolist(),
                         scores=[round(s, 2) for s in scores])
        if state == 'inconclusive':
            return self.moved
        self.pending = (self.pending + [state])[-self.confirm:]
        if len(self.pending) == self.confirm and len(set(self.pending)) == 1:
            self.moved = self.pending[0] == 'moved'
        return self.moved

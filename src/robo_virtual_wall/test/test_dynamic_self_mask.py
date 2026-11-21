from pathlib import Path
import cv2
import numpy as np
import pytest
from robo_virtual_wall.dynamic_self_mask import DynamicSelfMask
from robo_virtual_wall import detector

CONFIG = Path(__file__).resolve().parents[1] / 'config/dynamic_gripper_20261113.json'


def test_reference_acquisition_loss_and_appearance_reacquisition():
    tracker = DynamicSelfMask(CONFIG)
    assert tracker.update(tracker.reference, now=1.)
    assert all(part['inliers'] >= 6 for part in tracker.parts)
    assert not tracker.update(np.full_like(tracker.reference, 220), now=1.1)
    assert not tracker.status['valid']
    assert tracker.update(tracker.reference, now=1.2)
    assert tracker.status['valid']


def test_mask_moves_laterally_and_in_extension_instead_of_remaining_fixed():
    tracker = DynamicSelfMask(CONFIG)
    assert tracker.update(tracker.reference, now=1.)
    originals = [poly.copy() for poly in tracker.polygons]
    for index, (dx, dy) in enumerate([(8, -6), (16, -12), (24, -18)]):
        image = cv2.warpAffine(tracker.reference, np.float32([[1, 0, dx], [0, 1, dy]]),
                               (640, 480), borderMode=cv2.BORDER_REFLECT)
        assert tracker.update(image, now=1.1 + index*.1)
        for old, current in zip(originals, tracker.polygons):
            assert np.linalg.norm(np.mean(current-old, axis=0)-[dx, dy]) < 2.


def test_two_fingers_follow_different_movements_with_body_stationary():
    tracker = DynamicSelfMask(CONFIG)
    assert tracker.update(tracker.reference, now=1.)
    originals = [poly.copy() for poly in tracker.polygons]
    image = tracker.reference.copy()
    masks = []
    for part in tracker.parts:
        mask = np.zeros(image.shape[:2], np.uint8)
        cv2.fillPoly(mask, [part['polygon'].astype(int)], 255)
        masks.append(mask)
    union = np.maximum.reduce(masks)
    image[union != 0] = 190
    motions = [(-12, -6), (12, -6), (0, 0)]
    for mask, (dx, dy) in zip(masks, motions):
        matrix = np.float32([[1, 0, dx], [0, 1, dy]])
        piece = cv2.warpAffine(tracker.reference, matrix, (640, 480))
        support = cv2.warpAffine(mask, matrix, (640, 480))
        image[support != 0] = piece[support != 0]
    assert tracker.update(image, now=1.1)
    for old, current, expected in zip(originals, tracker.polygons, motions):
        assert np.linalg.norm(np.mean(current-old, axis=0)-expected) < 3.


def test_real_gripper_false_positive_removed_but_external_tape_preserved(monkeypatch):
    monkeypatch.setattr(detector, 'ROI_TOPO', 0.)
    tracker = DynamicSelfMask(CONFIG)
    image = tracker.reference.copy()
    # Tape is independent of the gripper appearance; do not mask by red hue.
    cv2.rectangle(image, (75, 70), (565, 94), (255, 255, 255), -1)
    for x in range(80, 545, 28):
        cv2.rectangle(image, (x, 72), (x+13, 92), (0, 0, 255), -1)
    assert tracker.update(image, now=1.)
    cfg = dict(h_min=170, h_max=10, s_min=45, s_max=255, v_min=40, v_max=255)
    mask, hsv = detector.mascara_fita(tracker.erase(image), cfg)
    mask, hsv = tracker.erase(mask), tracker.erase(hsv)
    mask[408:] = 0
    united = tracker.erase(detector.unir_tracejado(mask))
    united[408:] = 0
    segments = tracker.visible_segments(detector.detectar_paredes(united, mask, hsv))
    assert segments
    assert all(max(a[1], b[1]) < 110 for a, b, _ in segments)


def test_tape_alone_cannot_initialize_gripper_and_resolution_change_rejected():
    tracker = DynamicSelfMask(CONFIG)
    image = np.full_like(tracker.reference, 230)
    for y in range(120, 400, 25):
        cv2.rectangle(image, (270, y), (295, y+12), (0, 0, 255), -1)
    assert not tracker.update(image, now=1.)
    with pytest.raises(ValueError, match='resolution changed'):
        tracker.update(np.zeros((960, 1280, 3), np.uint8), now=1.1)


def test_current_pose_uses_only_robot_feature_support_with_real_tape_present():
    tracker = DynamicSelfMask(CONFIG.with_name('dynamic_gripper_current_20261113.json'))
    assert tracker.update(tracker.reference, now=1.)
    for index in range(1, 17):
        image = cv2.warpAffine(tracker.reference,
            np.float32([[1, 0, min(index, 8)], [0, 1, -min(index, 8)]]),
            (640, 480), borderMode=cv2.BORDER_REFLECT)
        assert tracker.update(image, now=1.+index*.1)
    assert all(part['inliers'] >= 6 for part in tracker.parts)

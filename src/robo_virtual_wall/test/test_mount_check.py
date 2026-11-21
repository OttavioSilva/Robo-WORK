"""CAMERA_MEXEU_20261121: real photos from 13/11 (camera repositioned ~21:25)."""
from pathlib import Path

import cv2
import numpy as np

from robo_virtual_wall.mount_check import MountCheck, choose_patches

FIXTURES = Path(__file__).parent / 'fixtures'


def load(name):
    return cv2.imread(str(FIXTURES / name), cv2.IMREAD_GRAYSCALE)


def check():
    reference = load('mount_reference.png')
    return reference, MountCheck(reference, choose_patches(reference))


def test_real_same_mount_ok_and_real_previous_mount_moved():
    _, mount = check()
    assert mount.measure(load('mount_same.png'))[0] == 'ok'
    assert mount.measure(load('mount_before_move.png'))[0] == 'moved'


def test_synthetic_moves_and_non_moves():
    reference, mount = check()
    size = reference.shape[::-1]
    move = lambda m: cv2.warpAffine(reference, m, size, borderMode=cv2.BORDER_REPLICATE)
    assert mount.measure(move(np.float32([[1, 0, 5], [0, 1, 0]])))[0] == 'moved'
    assert mount.measure(move(np.float32([[1, 0, 2], [0, 1, 1]])))[0] == 'ok'
    for angle in (1., -3.):
        assert mount.measure(move(cv2.getRotationMatrix2D((320, 240), angle, 1)))[0] == 'moved'
    x, y, w, h = choose_patches(reference)[0]
    covered = reference.copy()
    covered[y:y + h, x:x + w] = 30
    assert mount.measure(covered)[0] == 'inconclusive'
    blurred = reference.copy()
    blurred[y:y + h, x:x + w] = cv2.GaussianBlur(blurred[y:y + h, x:x + w], (21, 21), 0)
    assert mount.measure(blurred)[0] == 'ok'


def test_debounce_needs_consecutive_agreement():
    reference, mount = check()
    moved = load('mount_before_move.png')
    assert not mount.update(moved) and not mount.update(moved)
    assert mount.update(moved)
    covered = moved.copy()
    covered[300:] = 30
    assert mount.update(covered)            # inconclusive keeps the state
    for _ in range(2):
        assert mount.update(reference)
    assert not mount.update(reference)

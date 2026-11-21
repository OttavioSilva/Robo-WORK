import json
from pathlib import Path
import cv2
import numpy as np
import pytest
from robo_virtual_wall import detector
from robo_virtual_wall.self_mask import SelfMask

ROOT = Path(__file__).resolve().parents[1]


def detect(frame, model=None):
    cfg = dict(h_min=170, h_max=10, s_min=45, s_max=255, v_min=40, v_max=255)
    frame = model.erase(frame) if model else frame
    mask, hsv = detector.mascara_fita(frame, cfg)
    mask[408:] = 0
    if model:
        mask, hsv = model.erase(mask), model.erase(hsv)
    united = detector.unir_tracejado(mask)
    united[408:] = 0
    if model:
        united = model.erase(united)
    walls = detector.detectar_paredes(united, mask, hsv)
    return model.visible_segments(walls) if model else walls


def test_real_gripper_false_tape_removed_and_floor_tape_preserved(monkeypatch):
    monkeypatch.setattr(detector, 'ROI_TOPO', 0.)
    model = SelfMask(ROOT / 'config/self_mask_20261113.json')
    frame = cv2.imread(str(ROOT / 'test/fixtures/gripper_false_tape.png'))
    assert len(detect(frame)) == 1  # Recorded false positive, no tape on floor.
    assert detect(frame, model) == []
    # Add a synthetic ground stripe above the gripper; same red must survive.
    cv2.rectangle(frame, (70, 70), (570, 94), (255, 255, 255), -1)
    for x in range(75, 550, 28):
        cv2.rectangle(frame, (x, 72), (x+13, 92), (0, 0, 255), -1)
    walls = detect(frame, model)
    assert walls
    assert all(max(a[1], b[1]) < 110 for a, b, _ in walls)


def test_no_segment_bridges_robot_and_resolution_change_rejected():
    model = SelfMask(ROOT / 'config/self_mask_20261113.json')
    segments = model.visible_segments([((50, 300), (590, 300), 0.)])
    assert len(segments) == 2
    for a, b, _ in segments:
        points = np.rint(np.linspace(a, b, 100)).astype(int)
        assert not model.mask[points[:, 1], points[:, 0]].any()
    with pytest.raises(ValueError, match='resolution changed'):
        model.erase(np.zeros((960, 1280, 3), np.uint8))


def test_invalid_mask_rejected(tmp_path):
    path = tmp_path / 'invalid.json'
    path.write_text(json.dumps(dict(image_size=[640, 480], polygons=[[[0, 0], [640, 0], [1, 10]]])))
    with pytest.raises(ValueError, match='outside'):
        SelfMask(path)

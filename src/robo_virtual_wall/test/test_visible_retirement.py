from pathlib import Path
import cv2
import numpy as np
from robo_virtual_wall.visible_retirement import VisibleRetirement, observed_ground
from robo_virtual_wall.projection import load_calibration

WALL = np.array([[.8, -.2], [.8, .2]])
VIEW = np.array([[.4, -.5], [1.4, -.5], [1.4, .5], [.4, .5]])


def advance(retirement, walls, start=1., observations=(), view=VIEW, stationary=True, seconds=3.):
    removed = 0
    for stamp in np.arange(start, start+seconds, .1):
        removed += retirement.update(walls, list(observations), view, float(stamp), stationary)
    return removed


def test_removes_after_continuous_stationary_visible_absence():
    walls = [WALL.copy()]
    retirement = VisibleRetirement()
    assert not advance(retirement, walls, seconds=1.9)
    assert walls
    assert advance(retirement, walls, start=2.9, seconds=.5) == 1
    assert walls == []


def test_no_expiry_while_moving_out_of_view_or_without_image_evidence():
    for view, stationary in [(VIEW, False), (VIEW+10, True), (None, True)]:
        walls = [WALL.copy()]
        assert advance(VisibleRetirement(), walls, view=view, stationary=stationary, seconds=10.) == 0
        assert len(walls) == 1


def test_shorter_reversed_observation_preserves_long_wall():
    observation = np.array([[.805, .1], [.805, -.1]])
    walls = [WALL.copy()]
    assert not advance(VisibleRetirement(), walls, observations=[observation], seconds=10.)
    assert len(walls) == 1


def test_one_visible_wall_removed_while_other_remains_outside_view():
    walls = [WALL.copy(), WALL+[2, 0]]
    assert advance(VisibleRetirement(), walls) == 1
    assert len(walls) == 1
    assert np.allclose(walls[0], WALL+[2, 0])


def test_motion_dropout_and_positive_detection_reset_absence():
    for interruption in ('motion', 'view', 'positive', 'gap'):
        walls = [WALL.copy()]
        retirement = VisibleRetirement()
        advance(retirement, walls, seconds=1.8)
        if interruption == 'motion':
            retirement.update(walls, [], VIEW, 2.8, False)
        elif interruption == 'view':
            retirement.update(walls, [], None, 2.8, True)
        elif interruption == 'positive':
            retirement.update(walls, [WALL], VIEW, 2.8, True)
        else:
            retirement.update(walls, [], VIEW, 3.5, True)
        restart = 3.6 if interruption == 'gap' else 2.9
        assert not advance(retirement, walls, start=restart, seconds=1.8)
        assert walls


def test_duplicate_images_do_not_count_or_age_into_removal():
    walls = [WALL.copy()]
    retirement = VisibleRetirement(timeout=.2, min_frames=6)
    retirement.update(walls, [], VIEW, 1., True)
    for _ in range(20):
        assert retirement.update(walls, [], VIEW, 1., True) == 0
    retirement.update(walls, [], VIEW, 1.3, True)
    assert walls


def test_ground_support_requires_exact_resolution_quality_and_roi():
    root = Path(__file__).resolve().parents[1]
    calibration = load_calibration(root/'calibration/front_base_link_expanded_validated_20261113.json')
    frame = cv2.imread(str(root/'test/fixtures/gripper_false_tape.png'))
    polygon = observed_ground(frame, calibration, 0., .85, 3.)
    assert polygon is not None
    assert np.all(polygon[:, 0] > .365)
    assert observed_ground(np.zeros_like(frame), calibration, 0., .85, 3.) is None
    assert observed_ground(np.full_like(frame, 255), calibration, 0., .85, 3.) is None
    assert observed_ground(cv2.resize(frame, (1280, 960)), calibration, 0., .85, 3.) is None

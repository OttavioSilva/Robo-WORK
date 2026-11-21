import math
import numpy as np
from robo_virtual_wall.wall_map import WallTracker, rasterize
from robo_virtual_wall.projection import project_observed_segment


def test_confirmation_loss_reversal_and_persistence():
    wall = np.array([[.5,-.2],[.5,.2]])
    tracker = WallTracker(confirmations=3)
    assert not tracker.update([wall,wall[::-1]])
    assert not tracker.walls
    assert not tracker.update([])
    assert not tracker.update([wall])
    assert not tracker.update([wall[::-1]])
    assert tracker.update([wall])
    assert len(tracker.walls) == 1
    assert not tracker.update([])
    assert len(tracker.walls) == 1
    assert not tracker.update([wall+.001])
    assert len(tracker.walls) == 1


def test_capacity_and_invalid_segments():
    tracker = WallTracker(confirmations=2,max_walls=1)
    a,b = np.array([[0.,0.],[1.,0.]]),np.array([[0.,1.],[1.,1.]])
    tracker.update([a,b,[[float('nan'),0],[1,1]]])
    assert tracker.update([a,b])
    assert len(tracker.walls) == 1


def test_rasterization_respects_rotation_origin_and_edges():
    # In a 90-degree rotated grid, world x=1 is grid y=1 from origin (2,0).
    wall = np.array([[1.,.2],[1.,.8]])
    grid = rasterize([wall],20,20,.1,(2.,0.,math.pi/2),.02)
    assert grid[9:11,2:8].any()
    assert not grid[:5].any()
    assert not grid[:,15:].any()
    assert set(np.unique(grid)) == {0,100}
    assert not rasterize([wall+100],20,20,.1,(0,0,0),.02).any()
    # A segment crossing the entire map is clipped to bounds during rasterization.
    grid = rasterize([np.array([[-1.,.5],[4.,.5]])],20,20,.1,(0,0,0),.02)
    assert (grid[5] == 100).all()


def test_partly_visible_metric_segment_is_clipped_not_extrapolated():
    cal = (np.array([[.01,0,0],[0,.01,-.5],[0,0,1.]]),(100,100),
           np.float32([[20,20],[80,20],[80,80],[20,80]]))
    points = project_observed_segment(((40,0),(40,99)),cal,(100,100),3.)
    assert np.allclose(points,[[.4,-.3],[.4,.3]],atol=1e-5)
    assert project_observed_segment(((10,0),(10,99)),cal,(100,100),3.) is None


def test_base_calibration_requires_independent_distributed_checks():
    import pytest
    from robo_virtual_wall.calibrate import validated_base_calibration
    preview = dict(H=[[.01,0,0],[0,.01,-.5],[0,0,1]],
                   frame_id='virtual_wall_front',image_space='raw',
                   valid_pixels=[[20,20],[80,20],[80,80],[20,80]],
                   pixels=[[20,20],[80,20],[80,80],[20,80]],
                   metres=[[.2,-.3],[.8,-.3],[.8,.3],[.2,.3]])
    checks = dict(pixels=[[30,30],[70,30],[50,70]],
                  metres=[[.3,-.2],[.7,-.2],[.5,.2]])
    result = validated_base_calibration(preview,checks,.365,0.)
    assert result['validated'] and result['frame_id'] == 'base_link'
    h = np.asarray(result['H']) @ [30,30,1]
    assert np.allclose(h[:2]/h[2],[.665,-.2])
    bad = dict(checks,metres=[[.4,-.2],[.7,-.2],[.5,.2]])
    with pytest.raises(ValueError,match='exceeds'):
        validated_base_calibration(preview,bad,.365,0.)
    with pytest.raises(ValueError,match='three'):
        validated_base_calibration(preview,dict(pixels=[[30,30]],metres=[[.3,-.2]]),.365,0.)
    with pytest.raises(ValueError,match='reuse'):
        validated_base_calibration(preview,dict(pixels=[[20,20],[70,30],[50,70]],
                                             metres=[[.2,-.3],[.7,-.2],[.5,.2]]),.365,0.)


def test_preview_resizing_preserves_distance_and_rejects_aspect_change():
    import pytest
    from robo_virtual_wall.projection import resize_preview, preview_observation, project_observed_segment
    cal = (np.array([[.01,0,0],[0,.01,-.5],[0,0,1.]]),(100,100),
           np.float32([[20,20],[80,20],[80,80],[20,80]]))
    resized = resize_preview(cal,(200,200))
    assert np.allclose(resized[2],cal[2]*2)
    original = preview_observation(((40,20),(40,80)),cal,(100,100))
    enlarged = preview_observation(((80,40),(80,160)),resized,(200,200))
    assert enlarged['forward_m'] == pytest.approx(original['forward_m'])
    assert enlarged['pixel'] == tuple(2*v for v in original['pixel'])
    with pytest.raises(ValueError,match='aspect'):
        resize_preview(cal,(200,100))
    # Navigation keeps strict resolution validation even if preview resizing is on.
    with pytest.raises(ValueError,match='resolution'):
        project_observed_segment(((80,40),(80,160)),cal,(200,200),3.)


def test_recorded_field_checks_generate_base_link_calibration():
    import json
    import pytest
    from pathlib import Path
    from robo_virtual_wall.calibrate import base_calibration_from_field_checks
    root = Path(__file__).parents[1]/'calibration'
    preview = json.loads((root/'front_preview_20261106_repositioned.json').read_text())
    evidence = json.loads((root/'field_checks_20261106.json').read_text())
    result = base_calibration_from_field_checks(preview,evidence,.365,0.)
    pixel = np.array([398.,300.,1.])
    before,after = np.asarray(preview['H'])@pixel,np.asarray(result['H'])@pixel
    assert np.allclose(after[:2]/after[2]-before[:2]/before[2],[.365,0.])
    assert result['validated'] and not preview['validated']
    assert result['validation']['method'] == 'user_confirmed_field_screen_checks'
    with pytest.raises(ValueError,match='confirmation'):
        base_calibration_from_field_checks(preview,dict(evidence,user_confirmed=False),.365,0.)
    wrong = dict(evidence,longitudinal_checks=[dict(measured_m=.25,displayed_m=.40)]*3)
    with pytest.raises(ValueError,match='Longitudinal'):
        base_calibration_from_field_checks(preview,wrong,.365,0.)


def test_confirmation_uses_median_not_last_frame_and_reversed_endpoints():
    wall = np.array([[.7,-.15],[.7,.15]])
    tracker = WallTracker()
    tracker.update([wall])
    tracker.update([(wall+[.005,0])[::-1]])
    assert tracker.update([wall+[.015,0]])
    assert np.allclose(tracker.walls[0],wall+[.005,0])


def test_walk_and_tiny_edge_fragments_do_not_confirm():
    wall = np.array([[.7,-.15],[.7,.15]])
    tracker = WallTracker()
    for step in range(12):
        tracker.update([wall+[.02*step,0]])
    assert not tracker.walls
    tracker = WallTracker()
    for _ in range(10):
        tracker.update([np.array([[.7,0],[.7,.04]])])
    assert not tracker.walls


def test_shorter_observations_do_not_accumulate_duplicate_mask_walls():
    wall = np.array([[.7,-.3],[.7,.3]])
    tracker = WallTracker()
    for _ in range(3):
        tracker.update([wall])
    shorter = np.array([[.7,-.1],[.7,.1]])
    for _ in range(10):
        assert not tracker.update([shorter])
    assert len(tracker.walls) == 1
    assert np.array_equal(tracker.walls[0],wall)


# LINHA_FITA_20261121 -----------------------------------------------------------
def confirm(tracker, observations):
    for observation in observations:
        tracker.update([np.asarray(observation, float)])


def test_sliding_pieces_of_one_tape_confirm_while_moving():
    # Robot driving along a long tape: each frame sees a different 40 cm piece.
    tracker = WallTracker()
    pieces = [[[.6, -.6 + .02 * k], [.6, -.2 + .02 * k]] for k in range(31)]   # last piece ends at y=0.4
    confirm(tracker, pieces)
    assert len(tracker.walls) == 1
    wall = tracker.walls[0]
    low, high = sorted(wall[:, 1])
    assert abs(low + .6) < 1e-6                 # starts where the tape was first seen
    assert .3 <= high <= .4 + 1e-6               # grew with the drive, never past what was seen
    assert np.allclose(wall[:, 0], .6, atol=1e-6)


def test_lateral_walk_still_rejected_with_sliding_rule():
    tracker = WallTracker()
    confirm(tracker, [[[.6 + .02 * k, -.2], [.6 + .02 * k, .2]] for k in range(12)])
    assert not tracker.walls


def test_collinear_new_piece_extends_existing_wall():
    tracker = WallTracker()
    confirm(tracker, [[[.6, -.3], [.6, 0.]]] * 3)
    tracker.update([])
    confirm(tracker, [[[.605, -.05], [.605, .4]]] * 3)    # return pass, 5 mm off
    assert len(tracker.walls) == 1 and tracker.merged == 1
    assert np.allclose(sorted(tracker.walls[0][:, 1]), [-.3, .4], atol=1e-6)
    assert abs(tracker.walls[0][0, 0] - .6) < .006


def test_l_and_square_tapes_stay_separate_walls():
    tracker = WallTracker()
    square = [[[0., 0.], [.5, 0.]], [[.5, 0.], [.5, .5]], [[.5, .5], [0., .5]], [[0., .5], [0., 0.]]]
    for _ in range(3):
        tracker.update([np.array(s) for s in square])
    assert len(tracker.walls) == 4 and tracker.merged == 0
    assert not tracker.inconsistent


def test_rotated_copy_of_a_wall_is_kept_and_reported():
    tracker = WallTracker()
    confirm(tracker, [[[0., 0.], [1., 0.]]] * 3)
    tracker.update([])
    angle = math.radians(15)
    confirm(tracker, [[[0., 0.], [math.cos(angle), math.sin(angle)]]] * 3)
    assert len(tracker.walls) == 2              # conservative: both kept
    assert len(tracker.inconsistent) == 1
    assert abs(tracker.inconsistent[0]['angle_deg'] - 15) < .5


def test_ten_cm_merge_absorbs_slam_jump_but_not_rotated_copy():
    # FITA_ROBUSTA_20261121: node defaults (5 confirmations, 10 cm merge).
    tracker = WallTracker(confirmations=5, merge_tolerance=.10)
    confirm(tracker, [[[0., 0.], [1., 0.]]] * 5)
    tracker.update([])
    confirm(tracker, [[[.5, .08], [1.4, .08]]] * 5)      # same tape, pose jumped 8 cm
    assert len(tracker.walls) == 1 and not tracker.inconsistent
    tracker.update([])
    angle = math.radians(18)
    confirm(tracker, [[[0., 0.], [math.cos(angle), math.sin(angle)]]] * 5)
    assert len(tracker.walls) == 2 and len(tracker.inconsistent) == 1
    # Four frames of a look-alike are not enough for a new wall.
    confirm(tracker, [[[2., 2.], [2.3, 2.]]] * 4)
    assert len(tracker.walls) == 2


def test_walls_file_roundtrip_and_refusals(tmp_path):
    import json, pytest
    from robo_virtual_wall.wall_map import save_walls, load_walls
    walls = [np.array([[1., 2.], [1.5, 2.]]), np.array([[0., 0.], [0., .7]])]
    path = tmp_path / 'arena_fitas.json'
    assert save_walls(path, walls, 'map', 'teste') == 2
    loaded = load_walls(path, 'map')
    assert len(loaded) == 2 and np.allclose(loaded[0], walls[0])
    assert not (tmp_path / 'arena_fitas.json.tmp').exists()
    with pytest.raises(ValueError, match='frame'):
        load_walls(path, 'odom')
    data = json.loads(path.read_text())
    data['walls'].append([[0, 0], [float('nan'), 1]])
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='invalida'):
        load_walls(path, 'map')
    path.write_text(json.dumps(dict(format='outro', frame_id='map', walls=[])))
    with pytest.raises(ValueError, match='formato'):
        load_walls(path, 'map')

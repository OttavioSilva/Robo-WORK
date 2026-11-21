"""CALIBRACAO_CHARUCO_20261121: synthetic checks of lens + floor calibration."""
import json

import cv2
import numpy as np
import pytest

from robo_virtual_wall import charuco as C
from robo_virtual_wall.floor_calibration import (board_to_front, build_preview, calibrate_lens,
                                                 coverage_polygon, solve_floor)
from robo_virtual_wall.projection import calibration_from_dict, project_observed_segment, to_ground
from robo_virtual_wall.calibrate import validated_base_calibration

SIZE = (640, 480)
K = np.array([[520., 0., 322.], [0., 520., 236.], [0., 0., 1.]])
DIST = np.array([-0.28, 0.09, 0.0005, -0.0004, -0.012])


def camera_pose(height=0.36, back=0.10, tilt_deg=38., yaw_deg=1.5):
    """front->camera (R, t) for a camera above the robot looking forward/down."""
    t, y = np.radians(tilt_deg), np.radians(yaw_deg)
    z = np.array([np.cos(t) * np.cos(y), np.cos(t) * np.sin(y), -np.sin(t)])
    x = np.array([np.sin(y), -np.cos(y), 0.])
    yy = np.cross(z, x)
    to_front = np.column_stack((x, yy, z))
    centre = np.array([-back, 0.02, height])
    rotation = to_front.T
    return rotation, -rotation @ centre


def placements_for(boards, rotation, translation, dist, rng, pixel_noise=.15, tape_noise=.002):
    result = []
    for spec, a, b in boards:
        square = spec['square_mm'] / 1000
        corners_board = np.asarray(C.board(spec).chessboardCorners, float)
        measured_a = np.asarray(a) + rng.normal(0, tape_noise, 2)
        measured_b = np.asarray(b) + rng.normal(0, tape_noise, 2)
        true_r, true_t, _ = board_to_front(a, b, spec, square)
        floor = corners_board @ true_r.T + true_t
        rvec, _ = cv2.Rodrigues(rotation)
        pixels = cv2.projectPoints(floor, rvec, translation, K, dist)[0].reshape(-1, 2)
        pixels += rng.normal(0, pixel_noise, pixels.shape)
        r, t, length_error = board_to_front(measured_a, measured_b, spec, square)
        result.append(dict(name=spec['name'], corners=pixels, objects=corners_board,
                           rotation=r, translation=t, length_error=length_error))
    return result


# Edge A-B facing the robot: seen from the robot, A is on the right, B on the left.
BOARDS = [(C.FLOOR[0], [.20, -.11], [.20, .25]), (C.FLOOR[1], [.27, -.51], [.25, -.15]),
          (C.FLOOR[2], [.75, .04], [.75, .40]), (C.FLOOR[3], [.80, -.41], [.80, -.05])]


def test_board_frame_maps_corners_and_length():
    spec = C.FLOOR[0]
    r, t, error = board_to_front([.3, -.16], [.3, .2], spec, .06)
    assert abs(error) < 1e-9
    assert np.isclose(np.linalg.det(r), 1.)
    a, b = C.reference_points(spec, .06)
    assert np.allclose(r @ [*a, 0] + t, [.3, -.16, 0])
    assert np.allclose(r @ [*b, 0] + t, [.3, .2, 0])
    # A-B edge facing the robot (A right, B left): the board extends forward.
    assert (r @ [0, .1, 0])[0] > 0


def test_floor_pose_and_homography_recovered():
    rng = np.random.default_rng(1)
    rotation, translation = camera_pose()
    placements = placements_for(BOARDS, rotation, translation, DIST, rng)
    floor = solve_floor(placements, K, DIST)
    assert abs(floor['camera_height_m'] - .36) < .01
    assert abs(floor['tilt_down_deg'] - 38.) < 1.
    lens = dict(camera_matrix=K.tolist(), dist_coeffs=DIST.tolist(), rms_px=.2, views=20, rejected_views=0)
    data = build_preview(lens, floor, SIZE, [[5, 5], [630, 5], [630, 400], [5, 400]], 'test', [])
    cal = calibration_from_dict(json.loads(json.dumps(data)), preview=True)
    # Far and lateral points, well outside the boards, through raw pixels.
    rvec, _ = cv2.Rodrigues(rotation)
    truth = np.array([[x, y, 0.] for x in (.1, .5, 1., 1.6, 2.2) for y in (-.6, 0., .5)])
    raw = cv2.projectPoints(truth, rvec, translation, K, DIST)[0].reshape(-1, 2)
    inside = (raw[:, 0] > 0) & (raw[:, 0] < 639) & (raw[:, 1] > 0) & (raw[:, 1] < 479)
    g = to_ground(cal, raw[inside])
    error = np.linalg.norm(g[:, :2] / g[:, 2:] - truth[inside, :2], axis=1)
    assert inside.sum() >= 6
    assert error.max() < .02, error


def test_coverage_polygon_is_reliable_and_large():
    rng = np.random.default_rng(2)
    rotation, translation = camera_pose()
    floor = solve_floor(placements_for(BOARDS, rotation, translation, DIST, rng), K, DIST)
    lens = dict(camera_matrix=K.tolist(), dist_coeffs=DIST.tolist(), rms_px=.2, views=20, rejected_views=0)
    stub = build_preview(lens, floor, SIZE, [[5, 5], [630, 5], [630, 400], [5, 400]], 'test', [])
    polygon = coverage_polygon(calibration_from_dict(stub, preview=True), SIZE)
    assert cv2.isContourConvex(polygon.astype(np.float32))
    assert cv2.contourArea(polygon.astype(np.float32)) > .35 * SIZE[0] * SIZE[1] * .85
    data = dict(stub, valid_pixels=polygon.tolist())
    cal = calibration_from_dict(data, preview=True)
    ground = to_ground(cal, polygon)
    ground = ground[:, :2] / ground[:, 2:]
    assert (ground[:, 0] >= 0).all() and (np.hypot(ground[:, 0] + .365, ground[:, 1]) <= 2.96).all()


def test_promotion_with_lens_model_and_segment_projection():
    rng = np.random.default_rng(3)
    rotation, translation = camera_pose()
    floor = solve_floor(placements_for(BOARDS, rotation, translation, DIST, rng), K, DIST)
    lens = dict(camera_matrix=K.tolist(), dist_coeffs=DIST.tolist(), rms_px=.2, views=20, rejected_views=0)
    stub = build_preview(lens, floor, SIZE, [[5, 5], [630, 5], [630, 400], [5, 400]], 'test', [])
    preview = dict(stub, valid_pixels=coverage_polygon(calibration_from_dict(stub, True), SIZE).tolist())
    rvec, _ = cv2.Rodrigues(rotation)
    marks = np.array([[.35, .3, 0], [1.2, -.4, 0], [.9, .45, 0], [1.5, .1, 0]])
    raw = cv2.projectPoints(marks, rvec, translation, K, DIST)[0].reshape(-1, 2)
    checks = dict(pixels=raw.tolist(), metres=marks[:, :2].tolist(), image_size=list(SIZE))
    result = validated_base_calibration(preview, checks, .365, 0.)
    cal = calibration_from_dict(json.loads(json.dumps(result)))
    assert cal.undistorted and result['frame_id'] == 'base_link'
    # A straight tape from 0.5 m to 1.5 m ahead, measured from its raw endpoints.
    line = np.array([[.5, .1, 0], [1.5, .1, 0]])
    ends = cv2.projectPoints(line, rvec, translation, K, DIST)[0].reshape(-1, 2)
    metres = project_observed_segment(ends, cal, SIZE, 3.)
    assert metres is not None
    assert np.allclose(metres, line[:, :2] + [.365, 0], atol=.02)
    with pytest.raises(ValueError, match='exceeds'):
        validated_base_calibration(preview, dict(checks, metres=(marks[:, :2] + [.05, 0]).tolist()), .365, 0.)


def test_lens_calibration_recovers_intrinsics():
    rng = np.random.default_rng(4)
    board = C.board(C.LENS)
    objects = np.asarray(board.chessboardCorners, float)
    centre = objects.mean(axis=0)
    views = []
    while len(views) < 18:
        rvec = rng.normal(0, .35, 3)
        tvec = np.array([rng.uniform(-.12, .12), rng.uniform(-.08, .08), rng.uniform(.35, .6)])
        rotation, _ = cv2.Rodrigues(rvec)
        points = (objects - centre) @ rotation.T + tvec
        if (points[:, 2] <= .1).any():
            continue
        pixels = cv2.projectPoints(points, np.zeros(3), np.zeros(3), K, DIST)[0].reshape(-1, 2)
        inside = (pixels[:, 0] > 2) & (pixels[:, 0] < 637) & (pixels[:, 1] > 2) & (pixels[:, 1] < 477)
        if inside.sum() < 20:
            continue
        pixels = pixels + rng.normal(0, .1, pixels.shape)
        views.append((pixels[inside].astype(np.float32), np.flatnonzero(inside).astype(np.int32)))
    lens = calibrate_lens(views, SIZE)
    camera = np.asarray(lens['camera_matrix'])
    assert lens['rms_px'] < .5
    assert abs(camera[0, 0] - K[0, 0]) / K[0, 0] < .02
    assert abs(camera[0, 2] - K[0, 2]) < 6 and abs(camera[1, 2] - K[1, 2]) < 6


def render_floor(boards, rotation, translation):
    """Synthetic camera image (no distortion) with printed boards on a grey floor."""
    image = np.full(SIZE[::-1], 150, np.uint8)
    rvec, _ = cv2.Rodrigues(rotation)
    for spec, a, b in boards:
        sx, sy = spec['squares']
        px = 80
        art = C.board(spec).draw((sx * px, sy * px), marginSize=0, borderBits=1)
        art = cv2.copyMakeBorder(art, px // 2, px // 2, px // 2, px // 2, cv2.BORDER_CONSTANT, value=255)
        square = spec['square_mm'] / 1000
        r, t, _ = board_to_front(a, b, spec, square)
        # art pixel (u, v) -> board metres ((u - px/2) / px * square, (v - px/2) / px * square)
        art_pts = np.array([[0, 0], [art.shape[1], 0], [art.shape[1], art.shape[0]], [0, art.shape[0]]], float)
        board_pts = (art_pts - px / 2) / px * square
        floor = np.column_stack((board_pts, np.zeros(4))) @ r.T + t
        img = cv2.projectPoints(floor, rvec, translation, K, np.zeros(5))[0].reshape(-1, 2)
        warp = cv2.getPerspectiveTransform(art_pts.astype(np.float32), img.astype(np.float32))
        mask = cv2.warpPerspective(np.full(art.shape, 255, np.uint8), warp, SIZE)
        drawn = cv2.warpPerspective(art, warp, SIZE, flags=cv2.INTER_AREA)
        image[mask > 0] = drawn[mask > 0]
    return image


def test_rendered_image_detects_boards_with_correct_axes():
    rotation, translation = camera_pose(yaw_deg=0.)
    boards = [(C.FLOOR[0], [.25, -.06], [.25, .30]), (C.FLOOR[3], [.30, -.46], [.30, -.10])]
    image = render_floor(boards, rotation, translation)
    placements = []
    for spec, a, b in boards:
        found = C.detect(image, spec)
        assert found is not None, spec['name']
        corners, _, objects = found
        r, t, _ = board_to_front(a, b, spec, spec['square_mm'] / 1000)
        placements.append(dict(name=spec['name'], corners=corners, objects=objects, rotation=r, translation=t))
    floor = solve_floor(placements, K, np.zeros(5))
    assert floor['max_cm'] < 1.0, floor['per_board']
    assert abs(floor['camera_height_m'] - .36) < .01
    # Swapped A/B on one board must show up as a large per-board error.
    swapped = dict(placements[1])
    swapped['rotation'], swapped['translation'], _ = board_to_front([.30, -.10], [.30, -.46], C.FLOOR[3], .06)
    bad = solve_floor([placements[0], swapped], K, np.zeros(5))
    assert bad['max_cm'] > 5.


def test_ground_to_raw_inverts_projection():
    from robo_virtual_wall.floor_calibration import ground_to_raw
    rng = np.random.default_rng(5)
    rotation, translation = camera_pose()
    floor = solve_floor(placements_for(BOARDS, rotation, translation, DIST, rng), K, DIST)
    lens = dict(camera_matrix=K.tolist(), dist_coeffs=DIST.tolist(), rms_px=.2, views=20, rejected_views=0)
    cal = calibration_from_dict(build_preview(lens, floor, SIZE, [[5, 5], [630, 5], [630, 400], [5, 400]],
                                              't', []), preview=True)
    raw = np.array([[100., 300.], [320., 150.], [600., 380.]])
    g = to_ground(cal, raw)
    back = ground_to_raw(cal, g[:, :2] / g[:, 2:])
    assert np.allclose(back, raw, atol=.05)


def test_same_sheet_twice_is_refused():
    rotation, translation = camera_pose(yaw_deg=0.)
    boards = [(C.FLOOR[3], [.25, -.06], [.25, .30]), (C.FLOOR[3], [.30, -.46], [.30, -.10])]
    image = render_floor(boards, rotation, translation)
    with pytest.raises(C.DuplicateBoard, match='chao4'):
        C.detect(image, C.FLOOR[3])
    assert C.detect(image, C.FLOOR[0]) is None

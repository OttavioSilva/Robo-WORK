"""CALIBRACAO_CHARUCO_20261121: lens intrinsics + camera pose over the floor.

Frames
  board  : x along A->B, y across the board, z into the floor (as printed, face up)
  front  : x forward from the robot's white front edge, y left, z up
           (frame_id 'virtual_wall_front'; base_link = front + [0.365, 0])
The floor plane and camera pose come from PnP on printed boards; only each
board's position on the floor (corners A and B) is measured with a tape.
H then covers the whole visible floor, not just the boards: the usable area
is limited by pixel resolution and range, not by where marks were placed.
"""
import cv2
import numpy as np

from . import charuco


def calibrate_lens(views, image_size, spec=charuco.LENS, reject_factor=2.5):
    """views: list of (corners Nx2, ids N). Returns dict with K, dist, rms, per-view errors."""
    if len(views) < 10:
        raise ValueError('São necessárias pelo menos 10 fotos do tabuleiro da lente')
    board = charuco.board(spec)

    def run(selected):
        corners = [c.reshape(-1, 1, 2).astype(np.float32) for c, _ in selected]
        ids = [i.reshape(-1, 1).astype(np.int32) for _, i in selected]
        result = cv2.aruco.calibrateCameraCharucoExtended(corners, ids, board, tuple(image_size), None, None)
        rms, camera, dist, _, _, _, _, per_view = result
        return rms, camera, dist.ravel(), per_view.ravel()

    rms, camera, dist, per_view = run(views)
    limit = max(1.0, reject_factor * float(np.median(per_view)))
    kept = [v for v, e in zip(views, per_view) if e <= limit]
    rejected = len(views) - len(kept)
    if rejected and len(kept) >= 10:
        rms, camera, dist, per_view = run(kept)
    else:
        kept, rejected = views, 0
    if not np.isfinite(camera).all() or not np.isfinite(dist).all():
        raise ValueError('Calibração da lente não convergiu')
    return dict(camera_matrix=camera.tolist(), dist_coeffs=dist.tolist(), rms_px=float(rms),
                views=len(kept), rejected_views=rejected, per_view_px=[float(e) for e in per_view],
                image_size=list(image_size))


def board_to_front(a, b, spec, square_m):
    """Rigid board->front transform from measured corners A and B (metres).

    Returns (R 3x3, t 3, length error m). Board z (into the floor) maps to -z.
    """
    a, b = np.asarray(a, float), np.asarray(b, float)
    measured = np.linalg.norm(b - a)
    if measured < 1e-3:
        raise ValueError('A e B coincidem')
    true_length = spec['squares'][0] * square_m
    u = (b - a) / measured
    v = np.array([u[1], -u[0]])
    rotation = np.array([[u[0], v[0], 0.], [u[1], v[1], 0.], [0., 0., -1.]])
    return rotation, np.array([a[0], a[1], 0.]), float(measured - true_length)


def solve_floor(placements, camera_matrix, dist_coeffs):
    """placements: dicts with corners (raw px), objects (board m), rotation, translation, name.

    Joint PnP over all boards in the front frame. Returns pose and diagnostics.
    """
    camera_matrix = np.asarray(camera_matrix, float)
    dist_coeffs = np.asarray(dist_coeffs, float)
    objects, pixels, owners = [], [], []
    for index, item in enumerate(placements):
        floor = np.asarray(item['objects'], float) @ np.asarray(item['rotation']).T + item['translation']
        objects.append(floor)
        pixels.append(np.asarray(item['corners'], float))
        owners += [index] * len(floor)
    objects, pixels, owners = np.vstack(objects), np.vstack(pixels), np.asarray(owners)
    if len(objects) < 8:
        raise ValueError('Poucos cantos detectados no chão')
    ok, rvec, tvec = cv2.solvePnP(objects, pixels, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise ValueError('Não foi possível estimar a posição da câmera')
    rvec, tvec = cv2.solvePnPRefineLM(objects, pixels, camera_matrix, dist_coeffs, rvec, tvec)
    rotation, _ = cv2.Rodrigues(rvec)
    translation = tvec.ravel()
    projected = cv2.projectPoints(objects, rvec, tvec, camera_matrix, dist_coeffs)[0].reshape(-1, 2)
    reprojection = np.linalg.norm(projected - pixels, axis=1)
    H = homography_from_pose(camera_matrix, rotation, translation)
    model = cv2.undistortPoints(pixels.reshape(-1, 1, 2), camera_matrix, dist_coeffs,
                                P=camera_matrix).reshape(-1, 2)
    ground = np.column_stack((model, np.ones(len(model)))) @ H.T
    ground = ground[:, :2] / ground[:, 2:]
    error = np.linalg.norm(ground - objects[:, :2], axis=1)
    centre = -rotation.T @ translation
    axis = rotation.T @ np.array([0., 0., 1.])
    per_board = [dict(name=item['name'], corners=int((owners == i).sum()),
                      rms_cm=float(100 * np.sqrt(np.mean(error[owners == i] ** 2))),
                      max_cm=float(100 * error[owners == i].max()),
                      reprojection_px=float(np.sqrt(np.mean(reprojection[owners == i] ** 2))))
                 for i, item in enumerate(placements)]
    return dict(H=H, rotation=rotation, translation=translation, pixels=pixels,
                metres=objects[:, :2], per_board=per_board,
                camera_height_m=float(centre[2]), camera_position_m=centre.tolist(),
                tilt_down_deg=float(np.degrees(np.arcsin(-axis[2]))),
                yaw_deg=float(np.degrees(np.arctan2(axis[1], axis[0]))),
                rms_cm=float(100 * np.sqrt(np.mean(error ** 2))), max_cm=float(100 * error.max()))


def homography_from_pose(camera_matrix, rotation, translation):
    """Undistorted pixel -> front-frame floor metres, positive denominator in view."""
    forward = np.asarray(camera_matrix) @ np.column_stack((rotation[:, 0], rotation[:, 1], translation))
    # For a pixel (u, v, 1) of a visible floor point, inv(forward) gives
    # X/depth: the third component is 1/depth > 0. Scaling by a positive
    # number keeps that sign, as the projection code requires.
    H = np.linalg.inv(forward)
    return H / abs(H[2, 2]) if abs(H[2, 2]) > 1e-12 else H


def coverage_polygon(calibration, image_size, front_offset=0.365, max_range_m=3.0,
                     max_m_per_px=0.03, roi_bottom=0.85, min_forward_m=0.01, step=8):
    """Largest convex raw-pixel region whose floor points are reliable.

    Kept where: floor in front of the robot edge, base_link range below
    max_range_m (minus 5 cm), and one pixel moves the floor point at most
    max_m_per_px. Rows below roi_bottom (robot body) are excluded.
    """
    from .projection import to_ground
    width, height = image_size
    bottom = int(height * roi_bottom) - 1
    xs = np.arange(1, width - 1, step, dtype=float)
    ys = np.arange(1, bottom + 1, step, dtype=float)
    grid = np.array([[x, y] for y in ys for x in xs])

    def valid(points):
        points = np.asarray(points, float).reshape(-1, 2)
        result = np.zeros(len(points), bool)
        g = to_ground(calibration, points)
        right = to_ground(calibration, points + [1., 0.])
        down = to_ground(calibration, points + [0., 1.])
        for i, (a, r, d) in enumerate(zip(g, right, down)):
            if min(a[2], r[2], d[2]) <= 1e-9:
                continue
            p, pr, pd = a[:2] / a[2], r[:2] / r[2], d[:2] / d[2]
            base = np.hypot(p[0] + front_offset, p[1])
            resolution = max(np.linalg.norm(pr - p), np.linalg.norm(pd - p))
            result[i] = (p[0] >= min_forward_m and base <= max_range_m - .05
                         and resolution <= max_m_per_px and points[i][1] <= bottom)
        return result

    keep = valid(grid)
    if keep.sum() < 3:
        raise ValueError('Nenhuma região de chão confiável na imagem')
    hull = cv2.convexHull(grid[keep].astype(np.float32)).reshape(-1, 2).astype(float)
    centroid = hull.mean(axis=0)
    for _ in range(40):
        inside = np.array([cv2.pointPolygonTest(hull.astype(np.float32), tuple(p), False) >= 0
                           for p in grid])
        edges = np.vstack([np.linspace(a, b, 12) for a, b in zip(hull, np.roll(hull, -1, axis=0))])
        if valid(grid[inside]).all() and valid(edges).all():
            break
        hull = centroid + (hull - centroid) * .97
    else:
        raise ValueError('Região confiável não é convexa; confira as medidas')
    hull[:, 0] = np.clip(hull[:, 0], 0, width - 1)
    hull[:, 1] = np.clip(hull[:, 1], 0, height - 1)
    return hull


def build_preview(lens, floor, image_size, polygon, source, notes):
    """Front-frame preview JSON (validated=False), ready for independent checks."""
    return dict(H=np.asarray(floor['H']).tolist(), frame_id='virtual_wall_front', units='m',
                image_space='undistorted', validated=False, image_size=list(image_size),
                camera_matrix=lens['camera_matrix'], dist_coeffs=lens['dist_coeffs'],
                valid_pixels=np.asarray(polygon).tolist(),
                pixels=np.asarray(floor['pixels']).tolist(), metres=np.asarray(floor['metres']).tolist(),
                source=source, lens=dict((k, lens[k]) for k in ('rms_px', 'views', 'rejected_views')),
                floor_fit=dict(rms_cm=floor['rms_cm'], max_cm=floor['max_cm'], per_board=floor['per_board'],
                               camera_height_m=floor['camera_height_m'],
                               camera_position_m=floor['camera_position_m'],
                               tilt_down_deg=floor['tilt_down_deg'], yaw_deg=floor['yaw_deg']),
                notes=notes, validation=dict(status='pending_independent_checks', scope='preview_only'))


def ground_to_raw(calibration, points):
    """Floor metres (calibration frame) -> raw pixels, for overlays. NaN if behind."""
    matrix, _, _ = calibration
    points = np.asarray(points, float).reshape(-1, 2)
    model = np.column_stack((points, np.ones(len(points)))) @ np.linalg.inv(matrix).T
    result = np.full((len(points), 2), np.nan)
    front = model[:, 2] > 1e-9
    if not front.any():
        return result
    pixels = model[front, :2] / model[front, 2:]
    camera = getattr(calibration, 'camera_matrix', None)
    if camera is None:
        result[front] = pixels
        return result
    normal = np.column_stack(((pixels[:, 0] - camera[0, 2]) / camera[0, 0],
                              (pixels[:, 1] - camera[1, 2]) / camera[1, 1], np.ones(len(pixels))))
    raw = cv2.projectPoints(normal, np.zeros(3), np.zeros(3), camera, calibration.dist_coeffs)[0]
    result[front] = raw.reshape(-1, 2)
    return result

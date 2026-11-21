"""Validated planar projection. No implicit pixel/metre scale or legacy frame.

CALIBRACAO_CHARUCO_20261121: besides image_space 'raw' (H maps raw pixels),
'undistorted' calibrations carry camera_matrix/dist_coeffs and H maps
undistorted pixels (cv2.undistortPoints with P=K). Raw pixels stay the
interface everywhere: model_pixels() converts before applying H. Valid
polygons are always in raw pixels.
"""
import json
import cv2
import numpy as np

DIST_LENGTHS = (4, 5, 8, 12, 14)


class Calibration(tuple):
    """(H, image_size, raw polygon) plus an optional lens model.

    Still unpacks as the old 3-tuple, so existing call sites keep working.
    """
    def __new__(cls, matrix, size, polygon, camera_matrix=None, dist_coeffs=None):
        item = super().__new__(cls, (matrix, size, polygon))
        item.camera_matrix, item.dist_coeffs = camera_matrix, dist_coeffs
        return item

    @property
    def undistorted(self):
        return self.camera_matrix is not None


def model_pixels(calibration, pixels):
    """Raw pixels -> the pixel space H expects (identity for raw calibrations)."""
    points = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
    if getattr(calibration, 'camera_matrix', None) is None:
        return points
    return cv2.undistortPoints(points.reshape(-1, 1, 2), calibration.camera_matrix,
                               calibration.dist_coeffs, P=calibration.camera_matrix).reshape(-1, 2)


def to_ground(calibration, pixels):
    """Homogeneous ground coordinates (N x 3) of raw pixels."""
    model = model_pixels(calibration, pixels)
    return np.column_stack((model, np.ones(len(model)))) @ np.asarray(calibration[0]).T


def load_calibration(path, preview=False):
    with open(path, encoding='utf-8') as stream:
        data = json.load(stream)
    return calibration_from_dict(data, preview)


def calibration_from_dict(data, preview=False):
    expected_frame = 'virtual_wall_front' if preview else 'base_link'
    if data.get('frame_id') != expected_frame or data.get('units') != 'm':
        raise ValueError('Calibration requires frame_id=base_link and units=m')
    if (data.get('image_space') not in ('raw', 'undistorted')
            or (not preview and data.get('validated') is not True)):
        raise ValueError('Requires validated raw-image homography for this fixed mount')
    size = data.get('image_size', [])
    if len(size) != 2 or any(type(v) is not int or v <= 0 for v in size):
        raise ValueError('image_size must be [width, height]')
    matrix = np.asarray(data['H'], dtype=np.float64)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError('H must be a finite 3x3 matrix')
    if np.linalg.matrix_rank(matrix) != 3:
        raise ValueError('Singular homography')
    polygon = np.asarray(data['valid_pixels'], dtype=np.float32)
    if polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3:
        raise ValueError('valid_pixels requires a convex calibrated ground polygon')
    if not np.isfinite(polygon).all() or not cv2.isContourConvex(polygon):
        raise ValueError('Invalid calibration polygon')
    if (polygon < 0).any() or (polygon[:, 0] >= size[0]).any() or (polygon[:, 1] >= size[1]).any():
        raise ValueError('Calibration polygon outside image')
    if data['image_space'] == 'raw':
        return Calibration(matrix, tuple(size), polygon)
    camera = np.asarray(data.get('camera_matrix'), dtype=np.float64)
    dist = np.asarray(data.get('dist_coeffs'), dtype=np.float64).ravel()
    if (camera.shape != (3, 3) or not np.isfinite(camera).all() or camera[0, 0] <= 0
            or camera[1, 1] <= 0 or abs(camera[2, 2] - 1) > 1e-9):
        raise ValueError('camera_matrix must be a finite 3x3 intrinsic matrix')
    if len(dist) not in DIST_LENGTHS or not np.isfinite(dist).all():
        raise ValueError('dist_coeffs must have 4, 5, 8, 12 or 14 finite values')
    return Calibration(matrix, tuple(size), polygon, camera, dist)


def project_segment(segment, calibration, image_size, max_range):
    matrix, size, polygon = calibration
    if tuple(image_size) != size:
        raise ValueError('Image resolution differs from calibration')
    pixels = np.asarray(segment, dtype=np.float64)
    if pixels.shape != (2, 2) or not np.isfinite(pixels).all():
        return None
    if any(cv2.pointPolygonTest(polygon, tuple(p), False) < 0 for p in pixels):
        return None
    homogeneous = to_ground(calibration, pixels)
    denominator = homogeneous[:, 2]
    # Reject a segment crossing the projective horizon.
    if (np.abs(denominator) < 1e-9).any() or denominator[0] * denominator[1] <= 0:
        return None
    points = homogeneous[:, :2] / denominator[:, None]
    if not np.isfinite(points).all() or (points[:, 0] < 0).any():
        return None
    if (np.linalg.norm(points, axis=1) > max_range).any():
        return None
    return points


def centreline_intersection(segment, calibration, image_size, max_range=3.0):
    """Return (forward metres, pixel) at Y=0, only on observed valid support.

    Solve in homogeneous coordinates so perspective interpolation is exact.
    A longitudinal/parallel segment has no unique crossing and is rejected.
    """
    matrix, size, polygon = calibration
    if tuple(image_size) != size:
        raise ValueError('Image resolution differs from calibration')
    pixels = np.asarray(segment, dtype=float)
    if pixels.shape != (2, 2) or not np.isfinite(pixels).all():
        return None
    homogeneous = to_ground(calibration, pixels)
    denominator = homogeneous[:, 2]
    if (np.abs(denominator) < 1e-9).any() or denominator.prod() <= 0:
        return None
    delta_y = homogeneous[1, 1] - homogeneous[0, 1]
    if abs(delta_y) < 1e-9:
        return None
    t = -homogeneous[0, 1] / delta_y
    if not 0 <= t <= 1:
        return None
    pixel = pixels[0] + t * (pixels[1] - pixels[0])
    if cv2.pointPolygonTest(polygon, tuple(pixel), False) < 0:
        return None
    crossing = homogeneous[0] + t * (homogeneous[1] - homogeneous[0])
    forward = float(crossing[0] / crossing[2])
    if not np.isfinite(forward) or not 0 <= forward <= max_range:
        return None
    return forward, tuple(int(round(v)) for v in pixel)


def preview_observation(segment, calibration, image_size, max_range=3.0):
    """Classify current support in ground coordinates, without extrapolation.

    Clip to the calibrated convex polygon before projecting. Longitudinal tape
    gets a lateral offset rather than an ill-conditioned centreline distance.
    """
    matrix, size, polygon = calibration
    if tuple(image_size) != size:
        raise ValueError('Image resolution differs from calibration')
    points = np.asarray(segment, dtype=float)
    if points.shape != (2, 2) or not np.isfinite(points).all():
        return dict(kind='invalid')
    clipped = clip_segment(segment, polygon)
    if clipped is None:
        return dict(kind='outside')
    h = to_ground(calibration, clipped)
    if (np.abs(h[:, 2]) < 1e-9).any() or h[0, 2]*h[1, 2] <= 0:
        return dict(kind='invalid')
    ground = h[:, :2]/h[:, 2, None]
    if not np.isfinite(ground).all() or (ground[:, 0] < 0).any() or (np.linalg.norm(ground, axis=1) > max_range).any():
        return dict(kind='invalid')
    delta = ground[1]-ground[0]
    if np.linalg.norm(delta) < 1e-6:
        return dict(kind='invalid')
    if abs(delta[1]) <= abs(delta[0])*np.tan(np.deg2rad(20)):
        return dict(kind='longitudinal', lateral_m=float(ground[:, 1].mean()))
    crossing = centreline_intersection(segment, calibration, image_size, max_range)
    if crossing is None:
        return dict(kind='no_crossing')
    return dict(kind='crossing', forward_m=crossing[0], pixel=crossing[1])


def clip_segment(segment, polygon):
    points = np.asarray(segment, dtype=float)
    if points.shape != (2, 2) or not np.isfinite(points).all():
        return None
    direction = points[1] - points[0]
    # Intersect the observed segment with each polygon half-plane.
    orientation = np.sign(cv2.contourArea(polygon, oriented=True))
    lo, hi = 0., 1.
    for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
        edge = b-a
        cross = lambda v: orientation * (edge[0]*v[1]-edge[1]*v[0])
        start, slope = cross(points[0]-a), cross(direction)
        if abs(slope) < 1e-10:
            if start < -1e-7:
                return None
        elif slope > 0:
            lo = max(lo, -start/slope)
        else:
            hi = min(hi, -start/slope)
    if hi-lo < 1e-6:
        return None
    clipped = points[0] + np.array([lo, hi])[:, None]*direction
    return clipped


def project_observed_segment(segment, calibration, image_size, max_range):
    """Metric projection of only the observed support inside calibration."""
    if tuple(image_size) != calibration[1]:
        raise ValueError('Image resolution differs from calibration')
    clipped = clip_segment(segment, calibration[2])
    if clipped is None:
        return None
    # Numerical clipping residuals can place endpoints just outside OpenCV's
    # float32 polygon test. Nudge toward the midpoint by a subpixel fraction.
    middle = clipped.mean(axis=0)
    clipped = middle + (clipped-middle)*(1.-1e-6)
    return project_segment(clipped, calibration, image_size, max_range)


def resize_preview(calibration, image_size):
    """Tentative preview scaling; requires unchanged field of view and mount.

    Never use this helper to promote/resize validated navigation calibration.
    Equal aspect ratio is necessary, but cannot prove equal camera framing.
    """
    matrix, old_size, polygon = calibration
    if getattr(calibration, 'undistorted', False):
        raise ValueError('Lens-model calibration cannot be rescaled; recalibrate at this resolution')
    width,height = image_size
    if min(width,height) <= 0 or width*old_size[1] != height*old_size[0]:
        raise ValueError('Preview resize requires the same aspect ratio; recalibrate')
    sx,sy = width/old_size[0],height/old_size[1]
    return (matrix @ np.diag([1/sx,1/sy,1.]),tuple(image_size),
            np.asarray(polygon*np.array([sx,sy]),dtype=np.float32))

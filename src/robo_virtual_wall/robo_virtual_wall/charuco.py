"""CALIBRACAO_CHARUCO_20261121: board definitions shared by print and calibration.

Lens board: one A4 ChArUco (DICT_5X5_100), held by hand for intrinsics.
Floor boards: four A3 ChArUco, each with its own 12 ids of DICT_4X4_250, so
several can lie on the floor in the same photo and be told apart.
Sizes are the nominal printed sizes; the measured printed square size is
entered at calibration time (printers may scale).
"""
import cv2
import numpy as np

LENS = dict(name='lente', dictionary=cv2.aruco.DICT_5X5_100, squares=(9, 6),
            square_mm=28., marker_mm=21., first_id=0, paper='A4')
FLOOR = [dict(name='chao%d' % (k + 1), dictionary=cv2.aruco.DICT_4X4_250, squares=(6, 4),
              square_mm=60., marker_mm=45., first_id=12 * k, paper='A3') for k in range(4)]
PAPER_MM = dict(A4=(297., 210.), A3=(420., 297.))


def _dictionary(spec):
    full = cv2.aruco.getPredefinedDictionary(spec['dictionary'])
    count = (spec['squares'][0] * spec['squares'][1]) // 2
    sub = cv2.aruco.getPredefinedDictionary(spec['dictionary'])
    sub.bytesList = full.bytesList[spec['first_id']:spec['first_id'] + count]
    return full, sub, count


def board(spec, square_m=None):
    """CharucoBoard with local ids 0..n-1; square_m overrides the nominal size."""
    _, sub, _ = _dictionary(spec)
    square = (square_m if square_m is not None else spec['square_mm'] / 1000.)
    marker = square * spec['marker_mm'] / spec['square_mm']
    return cv2.aruco.CharucoBoard_create(spec['squares'][0], spec['squares'][1], square, marker, sub)


class DuplicateBoard(ValueError):
    """The same board id range is visible twice (sheet printed twice)."""
    def __init__(self, name):
        super().__init__('A folha %s aparece duas vezes na imagem: cada folha no chão '
                         'precisa ser diferente (chao1, chao2, chao3, chao4).' % name)
        self.name = name


def _detect_at(gray, spec, square_m, scale):
    full, _, count = _dictionary(spec)
    parameters = cv2.aruco.DetectorParameters_create()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    image = gray if scale == 1 else cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
    corners, ids, _ = cv2.aruco.detectMarkers(image, full, parameters=parameters)
    if ids is None:
        return 0, None, None
    ids = ids.ravel()
    keep = (ids >= spec['first_id']) & (ids < spec['first_id'] + count)
    if keep.sum() < 2:
        return 0, None, None
    if len(np.unique(ids[keep])) < keep.sum():
        # Same sheet printed twice: corners would mix two placements.
        raise DuplicateBoard(spec['name'])
    local = (ids[keep] - spec['first_id']).astype(np.int32).reshape(-1, 1)
    # Marker corners back to original pixels; ChArUco corners are refined there.
    chosen = [(corners[i] / scale).astype(np.float32) for i in np.flatnonzero(keep)]
    found, charuco, charuco_ids = cv2.aruco.interpolateCornersCharuco(chosen, local, gray, board(spec, square_m))
    return (found if charuco is not None else 0), charuco, charuco_ids


def detect(gray, spec, square_m=None, min_corners=6):
    """Charuco corners of one board: (corners Nx2 float32, ids N, object points Nx3) or None.

    Raises DuplicateBoard if the same sheet is seen twice.

    Small or distant boards (markers ~15 px) are retried on a 2x upscaled
    image, which on real floor photos doubled the detected markers.
    """
    total = (spec['squares'][0] - 1) * (spec['squares'][1] - 1)
    found, charuco, charuco_ids = _detect_at(gray, spec, square_m, 1)
    if found < total:
        retry = _detect_at(gray, spec, square_m, 2)
        if retry[0] > found:
            found, charuco, charuco_ids = retry
    if charuco is None or found < min_corners:
        return None
    b = board(spec, square_m)
    charuco_ids = charuco_ids.ravel()
    objects = np.asarray(b.chessboardCorners, dtype=np.float64)[charuco_ids]
    return charuco.reshape(-1, 2).astype(np.float32), charuco_ids, objects


def reference_points(spec, square_m=None):
    """Board-frame outer corners A and B printed as tape-measure references.

    A is the board origin (0,0); B is (squaresX*square, 0). Both lie on the
    edge labelled 'LADO DO ROBO' on the print.
    """
    square = square_m if square_m is not None else spec['square_mm'] / 1000.
    return np.array([[0., 0.], [spec['squares'][0] * square, 0.]])

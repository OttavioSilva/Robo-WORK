"""CORRIGE_POSE_MESA (24/11): pose do robo pela face cadastrada + medicao do face_ws (sem ROS)."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import face_ws  # noqa: E402

_SRC = open(os.path.join(os.path.dirname(__file__), '..', 'scripts', 'alinhador_passagem.py')).read()
_NS = {'math': math}
exec(_SRC[_SRC.index('def pose_pela_face'):_SRC.index('def giro_livre')], _NS)
pose_pela_face = _NS['pose_pela_face']


def _medir(face, px, py, pt):
    ux, uy = math.cos(face['yaw']), math.sin(face['yaw'])
    pts = []
    for k in range(-40, 41):
        wx, wy = face['x'] - uy * k * 0.01, face['y'] + ux * k * 0.01
        dx, dy = wx - px, wy - py
        c, s = math.cos(pt), math.sin(pt)
        pts.append((c * dx + s * dy, -s * dx + c * dy))
    return max((m for m in face_ws.candidatos_face(pts) if m['centro'] is not None), key=lambda m: m['n'])


def test_devolve_pose_real_varias_mesas():
    for face, (px, py, pt) in [
            (dict(x=0.5, y=8.748, yaw=math.pi), (1.005, 8.788, math.radians(183))),
            (dict(x=3.15, y=4.513, yaw=math.pi / 2), (3.13, 4.033, math.radians(86))),
            (dict(x=2.31, y=0.525, yaw=0.0), (1.81, 0.465, math.radians(2)))]:
        m = _medir(face, px, py, pt)
        x, y, yaw = pose_pela_face(face, m['dist'], m['ang'], m['centro'])
        assert math.hypot(x - px, y - py) < 0.002
        assert abs(math.atan2(math.sin(yaw - pt), math.cos(yaw - pt))) < math.radians(0.1)


def test_sem_centro_corrige_so_profundidade():
    face = dict(x=0.5, y=8.748, yaw=math.pi)
    m = _medir(face, 0.975, 8.80, math.pi)
    x, y, _ = pose_pela_face(face, m['dist'], m['ang'], None, (0.70, 8.75, math.pi))
    assert abs(x - 0.975) < 0.002      # profundidade pela mesa
    assert abs(y - 8.75) < 0.002       # ao longo da face: fica o do AMCL
    assert pose_pela_face(face, m['dist'], m['ang'], None, None) is None

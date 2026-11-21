"""ROS-independent confirmation and conservative rasterization of tape walls."""
import math
import numpy as np


class WallTracker:
    """Confirm straight tape walls from consecutive metric observations.

    LINHA_FITA_20261121: a candidate is a LINE (direction + lateral offset),
    not a pair of endpoints. While the robot moves, the camera sees sliding
    pieces of the same tape; they confirm the candidate if they stay on its
    line (lateral <= confirmation_tolerance from the first sample and from the
    current estimate, angle <= 10 deg) and touch its observed extent. The
    confirmed wall spans the union of observed pieces (never extrapolated).
    A confirmed piece that is collinear with an existing wall (angle <= 10
    deg, lateral <= merge_tolerance) and touches it EXTENDS that wall instead
    of adding another one; L/T/square tapes (90 deg) stay separate walls.
    Nearly parallel but clearly offset copies (e.g. a SLAM correction between
    passes) are kept and reported in self.inconsistent, never silently merged.
    """
    def __init__(self, confirmations=3, tolerance=.08, max_walls=2000,
                 confirmation_tolerance=.025, min_length=.08, merge_tolerance=.05,
                 angle_tolerance_deg=10.):
        self.confirmations = confirmations
        self.tolerance = tolerance
        self.max_walls = max_walls
        self.confirmation_tolerance = confirmation_tolerance
        self.min_length = min_length
        self.merge_tolerance = merge_tolerance
        self.cos_angle = math.cos(math.radians(angle_tolerance_deg))
        self.pending = []
        self.walls = []
        self.merged = 0
        self.inconsistent = []

    # ---------------------------------------------------------------- geometry
    @staticmethod
    def frame(segment):
        """origin, unit axis, unit normal of a segment."""
        axis = segment[1] - segment[0]
        axis = axis / np.linalg.norm(axis)
        return segment[0], axis, np.array([-axis[1], axis[0]])

    def same_direction(self, a, b):
        u, v = a[1] - a[0], b[1] - b[0]
        denominator = np.linalg.norm(u) * np.linalg.norm(v)
        return denominator > 1e-9 and abs(u @ v) / denominator >= self.cos_angle

    def lateral(self, points, segment):
        origin, _, normal = self.frame(segment)
        return np.abs((np.asarray(points) - origin) @ normal).max()

    def gap(self, points, segment):
        """Distance along the segment axis between the two extents (0 if overlapping)."""
        origin, axis, _ = self.frame(segment)
        along = (np.asarray(points) - origin) @ axis
        length = np.linalg.norm(segment[1] - segment[0])
        return max(0., along.min() - length, -along.max())

    def covered(self, points, wall):
        if not self.same_direction(points, wall):
            return False
        origin, axis, _ = self.frame(wall)
        along = (points - origin) @ axis
        length = np.linalg.norm(wall[1] - wall[0])
        return (self.lateral(points, wall) <= self.confirmation_tolerance and
                along.min() >= -self.tolerance and along.max() <= length + self.tolerance)

    @staticmethod
    def aligned(points, reference):
        if np.linalg.norm(points-reference,axis=1).max() > np.linalg.norm(points-reference[::-1],axis=1).max():
            return points[::-1]
        return points

    def fit(self, segments, reference):
        """One segment along the common line, spanning the union of segments.

        Direction: mean of aligned directions; lateral offset: median of all
        endpoints (robust to one outlier frame).
        """
        segments = [self.aligned(np.asarray(s, float), reference) for s in segments]
        directions = [(s[1] - s[0]) / np.linalg.norm(s[1] - s[0]) for s in segments]
        axis = np.mean(directions, axis=0)
        axis /= np.linalg.norm(axis)
        normal = np.array([-axis[1], axis[0]])
        points = np.vstack(segments)
        offset = np.median(points @ normal)
        along = points @ axis
        base = offset * normal
        return np.array([base + axis * along.min(), base + axis * along.max()])

    # ------------------------------------------------------------------ update
    def merge_into_walls(self, candidate):
        """Extend a collinear, touching wall; returns True if merged."""
        for index, wall in enumerate(self.walls):
            if (self.same_direction(candidate, wall)
                    and self.lateral(candidate, wall) <= self.merge_tolerance
                    and self.lateral(wall, candidate) <= self.merge_tolerance
                    and self.gap(candidate, wall) <= self.tolerance):
                length = np.linalg.norm(wall[1] - wall[0])
                pieces = [wall] * max(1, int(round(length / max(np.linalg.norm(candidate[1] - candidate[0]), 1e-3))))
                self.walls[index] = self.fit(pieces + [candidate], wall)
                self.merged += 1
                # The extended wall may now touch another one: merge those too.
                merged = self.walls.pop(index)
                if not self.merge_into_walls(merged):
                    self.walls.insert(index, merged)
                return True
        return False

    def note_inconsistency(self, candidate):
        """Record a new wall that looks like a displaced copy of an existing one.

        Copy = touching an existing wall, and either rotated 10-30 deg or
        parallel but laterally offset beyond merge_tolerance (up to 40 cm).
        Typical cause: localisation changed between two passes over one tape.
        """
        u = (candidate[1] - candidate[0]) / np.linalg.norm(candidate[1] - candidate[0])
        for wall in self.walls:
            v = (wall[1] - wall[0]) / np.linalg.norm(wall[1] - wall[0])
            cosine = abs(float(u @ v))
            if cosine < math.cos(math.radians(30)) or self.gap(candidate, wall) > self.tolerance:
                continue
            offset = self.lateral(candidate, wall)
            if offset > .4:
                continue
            if cosine < self.cos_angle or offset > self.merge_tolerance:
                self.inconsistent.append(dict(angle_deg=float(np.degrees(np.arccos(min(1., cosine)))),
                                              offset_m=float(offset), new=candidate.copy(), old=wall.copy()))
                return

    def update(self, segments):
        available = list(self.pending)
        pending = []
        added = False
        for points in segments:
            points = np.asarray(points, dtype=float)
            if points.shape != (2,2) or not np.isfinite(points).all():
                continue
            if np.linalg.norm(points[1]-points[0]) < self.min_length:
                continue
            if any(self.covered(points, wall) for wall in self.walls):
                continue
            if any(self.same_direction(points, old['line']) and
                   self.lateral(points, old['line']) <= self.confirmation_tolerance and
                   self.gap(points, old['line']) <= self.tolerance for old in pending):
                continue
            candidate = None
            for i, old in enumerate(available):
                first, line = old['samples'][0], old['line']
                # Stay on the line of the FIRST sample and of the current
                # estimate: pairwise association alone would let a drifting
                # candidate walk sideways.
                if (self.same_direction(points, line) and self.same_direction(points, first)
                        and self.lateral(points, line) <= self.confirmation_tolerance
                        and self.lateral(points, first) <= self.confirmation_tolerance
                        and self.gap(points, line) <= self.tolerance):
                    candidate = available.pop(i)
                    break
            samples = (candidate['samples'] if candidate else []) + [self.aligned(points, candidate['line'])
                                                                    if candidate else points.copy()]
            line = self.fit(samples, samples[0])
            if len(samples) >= self.confirmations:
                if self.merge_into_walls(line):
                    added = True
                elif len(self.walls) < self.max_walls:
                    self.note_inconsistency(line)
                    self.walls.append(line)
                    added = True
            else:
                pending.append(dict(samples=samples, line=line))
        self.pending = pending
        return added


def rasterize(walls, width, height, resolution, origin, half_width):
    """Mark every cell potentially intersected by the thick tape segment.

    half_width includes half tape width plus metric uncertainty, not footprint.
    Half a cell diagonal is added conservatively for raster discretization.
    Origin is (x, y, yaw); rotated OccupancyGrid origins are supported.
    """
    if width <= 0 or height <= 0 or resolution <= 0 or half_width < 0:
        raise ValueError('Invalid mask geometry')
    grid = np.zeros((height,width),dtype=np.int8)
    x,y,yaw = origin
    rotation = np.array([[math.cos(yaw),-math.sin(yaw)],
                         [math.sin(yaw), math.cos(yaw)]])
    radius = half_width/resolution + math.sqrt(.5)
    for wall in walls:
        points = (np.asarray(wall)-[x,y]) @ rotation / resolution
        lower = np.maximum(np.floor(points.min(axis=0)-radius).astype(int),0)
        upper = np.minimum(np.ceil(points.max(axis=0)+radius).astype(int),[width-1,height-1])
        if (upper < lower).any():
            continue
        xx,yy = np.meshgrid(np.arange(lower[0],upper[0]+1)+.5,
                            np.arange(lower[1],upper[1]+1)+.5)
        cells = np.stack((xx,yy),axis=-1)
        axis = points[1]-points[0]
        length2 = float(axis @ axis)
        if length2 < 1e-12:
            continue
        t = np.clip(((cells-points[0]) @ axis)/length2,0,1)
        distance = np.linalg.norm(cells-(points[0]+t[...,None]*axis),axis=-1)
        patch = grid[lower[1]:upper[1]+1,lower[0]:upper[0]+1]
        patch[distance <= radius] = 100
    return grid


# PAREDES_SALVAS_20261121 ------------------------------------------------------
WALLS_FORMAT = 'robo_virtual_wall/paredes_fita/v1'


def save_walls(path, walls, frame_id, note=''):
    """Atomically write confirmed walls (map frame) to JSON.

    Walls are only valid in the SAME map they were recorded in (fixed arena
    map in localization). The file stores the frame and a note to check it.
    """
    import json
    import os
    import time
    from pathlib import Path
    path = Path(path)
    data = dict(format=WALLS_FORMAT, frame_id=frame_id, note=note,
                saved=time.strftime('%Y-%m-%d %H:%M:%S'),
                walls=[np.asarray(w, float).round(4).tolist() for w in walls])
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(data, indent=1) + '\n', encoding='utf-8')
    os.replace(temporary, path)
    return len(data['walls'])


def load_walls(path, frame_id):
    """Walls from a file written by save_walls; refuses another frame or bad data."""
    import json
    from pathlib import Path
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    if data.get('format') != WALLS_FORMAT:
        raise ValueError('Arquivo de paredes com formato desconhecido')
    if data.get('frame_id') != frame_id:
        raise ValueError('Paredes gravadas no frame %r, mapa atual em %r' % (data.get('frame_id'), frame_id))
    walls = []
    for item in data.get('walls', []):
        wall = np.asarray(item, dtype=float)
        if wall.shape != (2, 2) or not np.isfinite(wall).all() or np.linalg.norm(wall[1] - wall[0]) < 1e-3:
            raise ValueError('Parede invalida no arquivo')
        walls.append(wall)
    return walls

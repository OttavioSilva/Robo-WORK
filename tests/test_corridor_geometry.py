#!/usr/bin/env python3
"""Testes sinteticos de corredor; executaveis sem ROS."""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from geometry_corridor_geometry import Grid, stage_for_path, disc_is_mapped_free


def example(angle=0.0, walls=True):
    res, nx, ny = 0.025, 240, 240
    ox, oy = -2.5, -2.5
    c, s = math.cos(angle), math.sin(angle)
    data = []
    for j in range(ny):
        y = oy + (j + 0.5) * res
        for i in range(nx):
            x = ox + (i + 0.5) * res
            longitudinal = c * x + s * y
            lateral = -s * x + c * y
            occupied = (
                walls and
                1.0 <= longitudinal <= 4.0 and
                0.385 <= abs(lateral) <= 0.430
            )
            data.append(100 if occupied else 0)
    g = Grid(nx, ny, res, ox, oy, 0.0, data)
    path = [(c * (0.05 * i), s * (0.05 * i)) for i in range(67)]
    return g, path


class CorridorTests(unittest.TestCase):
    def test_straight_corridor(self):
        g, path = example(0.0)
        result = stage_for_path(path, g)
        self.assertGreater(result["estimated_width"], 0.72)
        self.assertLess(result["estimated_width"], 1.05)
        self.assertAlmostEqual(result["corridor_heading"], 0.0, places=1)
        self.assertLess(result["stage"][0], result["entrance"][0] - 0.6)

    def test_rotated_corridor(self):
        theta = math.radians(32)
        g, path = example(theta)
        result = stage_for_path(path, g)
        self.assertAlmostEqual(result["corridor_heading"], theta, delta=0.15)
        self.assertGreater(result["distance_stage_to_entrance"], 0.64)

    def test_open_area_without_corridor(self):
        g, path = example(walls=False)
        with self.assertRaises(ValueError):
            stage_for_path(path, g)

    def test_unknown_area_rejects_stage(self):
        g, path = example()
        g.cells = [-1] * len(g.cells)
        with self.assertRaises(ValueError):
            stage_for_path(path, g)

    def test_disc_conflict(self):
        g, path = example()
        self.assertTrue(disc_is_mapped_free(g, 0.0, 0.0))
        self.assertFalse(disc_is_mapped_free(g, 1.3, 0.0))


if __name__ == "__main__":
    unittest.main()

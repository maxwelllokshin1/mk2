import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'follow_the_gap'))
from track_geometry import WallTracker, front_speed_factor, side_distance, wall_steer  # noqa: E402


def corridor_scan(left, right, n=720, far=10.0):
    """A straight corridor, car at (0, 0) facing +x; left wall at y = +left, right wall at y = -right."""
    angles = np.linspace(-math.pi, math.pi, n, endpoint=False)
    ranges = np.full(n, far)
    for i, a in enumerate(angles):
        s = math.sin(a)
        if s > 1e-6:
            ranges[i] = min(far, left / s)
        elif s < -1e-6:
            ranges[i] = min(far, -right / s)
    return angles, ranges


def test_sideways_distance_not_the_long_view_down_the_track():
    a, r = corridor_scan(0.6, 0.4)
    assert abs(side_distance(a, r, +1) - 0.6) < 0.02
    assert abs(side_distance(a, r, -1) - 0.4) < 0.02


def test_corridor_width_and_offset():
    a, r = corridor_scan(0.9, 0.5)
    w = WallTracker(alpha=1.0)
    w.update(a, r)
    assert abs(w.corridor - 1.4) < 0.05
    assert w.offset > 0                       # more room on the left


def test_tracks_changes_within_a_few_scans():
    w = WallTracker(alpha=0.5)
    w.update(*corridor_scan(1.0, 1.0))
    for _ in range(6):
        w.update(*corridor_scan(0.5, 1.5))
    assert abs(w.left - 0.5) < 0.05 and abs(w.right - 1.5) < 0.05


def test_open_side_reads_far_and_missing_side_keeps_last():
    w = WallTracker(alpha=1.0, open_m=10.0)
    a, r = corridor_scan(0.5, 0.5)
    w.update(a, r)
    r2 = r.copy()
    r2[(np.degrees(a) > 30) & (np.degrees(a) < 150)] = float('inf')    # left side sees nothing
    w.update(a, r2)
    assert abs(w.left - 0.5) < 0.05 and abs(w.right - 0.5) < 0.05


def test_front_speed_factor():
    assert front_speed_factor(2.0, 0.3, 1.5) == 1.0
    assert front_speed_factor(0.3, 0.3, 1.5) == 0.0
    assert 0.4 < front_speed_factor(0.9, 0.3, 1.5) < 0.6
    assert front_speed_factor(0.1, 0.3, 1.5, floor=0.1) == 0.1


def test_wall_steer_pushes_away_and_centres():
    m = 0.5
    assert wall_steer(0.2, 1.0, 0.6, m, 0.3, 0.25) < 0       # wall close on the left -> steer right
    assert wall_steer(1.0, 0.2, 0.6, m, 0.3, 0.25) > 0       # wall close on the right -> steer left
    assert abs(wall_steer(0.6, 0.6, 0.6, m, 0.3, 0.25)) < 1e-9
    assert wall_steer(0.9, 0.5, 0.7, m, 0.3, 0.25) > 0       # drifted right of centre -> gentle push left
    assert wall_steer(0.9, 0.5, 0.7, m, 0.3, 0.25, curve_frac=1.0) == 0.0   # in a bend: leave it to the gap logic
    assert wall_steer(None, 1.0, 0.6, m, 0.3, 0.25) == 0.0

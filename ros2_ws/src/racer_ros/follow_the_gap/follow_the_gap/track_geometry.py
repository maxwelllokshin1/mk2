"""
track_geometry.py: how far the car is from the walls on its LEFT and RIGHT, measured on every scan.
Pure numpy (no ROS) so tests can drive it.

The old track-size estimate averaged every beam within +-90 deg. Beams pointing down the track see 10 m of corridor, so
the average said "huge track" almost everywhere, and the bubble / speed / range were all scaled up for a track that is
really 1-2 m wide. Here the width is the SIDEWAYS distance to each wall: a beam of length r at angle a from straight
ahead is r * |sin(a)| away from the car's centre line, and the nearest wall on a side is a low percentile of those.

    left, right = walls.update(angles, ranges)        # metres, smoothed
    walls.corridor, walls.offset                      # left + right, (left - right) / 2
"""
import numpy as np


def side_distance(angles, ranges, side, lo_deg=50.0, hi_deg=130.0, range_min=0.05, range_max=10.0, percentile=15.0):
    """Sideways distance (m) from the car's centre line to the nearest wall on one side (+1 = left, -1 = right).
    Looks at beams lo_deg..hi_deg off straight ahead on that side. None when that side has no usable beams."""
    a = np.asarray(angles, dtype=float)
    r = np.asarray(ranges, dtype=float)
    off = np.degrees(a) * side                                  # degrees off straight ahead, positive on this side
    ok = (off >= lo_deg) & (off <= hi_deg) & np.isfinite(r) & (r > range_min) & (r < range_max)
    if np.count_nonzero(ok) < 3:
        return None
    lateral = r[ok] * np.abs(np.sin(a[ok]))
    return float(np.percentile(lateral, percentile))


class WallTracker:
    """Left / right wall distance, smoothed with an exponential average. alpha 1.0 = no smoothing."""

    def __init__(self, alpha=0.3, open_m=10.0, **window):
        self.alpha, self.open_m, self.window = alpha, open_m, window
        self.left = self.right = None

    def update(self, angles, ranges):
        for name, side in (('left', 1), ('right', -1)):
            d = side_distance(angles, ranges, side, range_max=self.open_m, **self.window)
            if d is None:
                continue                                        # nothing seen this scan: keep the last value
            old = getattr(self, name)
            setattr(self, name, d if old is None else (1.0 - self.alpha) * old + self.alpha * d)
        return self.left, self.right

    def reset(self):
        self.left = self.right = None

    @property
    def known(self):
        return self.left is not None and self.right is not None

    @property
    def corridor(self):
        return None if not self.known else self.left + self.right

    @property
    def offset(self):
        """+ = more room on the left than the right (car sits right of centre)."""
        return None if not self.known else (self.left - self.right) / 2.0


def front_speed_factor(front_m, stop_m, slow_m, floor=0.0):
    """1.0 while the way ahead is clear (>= slow_m), falling linearly to `floor` at stop_m and closer."""
    if slow_m <= stop_m:
        return 1.0 if front_m >= stop_m else floor
    f = (front_m - stop_m) / (slow_m - stop_m)
    return float(min(1.0, max(floor, f)))


def wall_steer(left, right, half_corridor, max_steer, center_gain, danger_m, curve_frac=0.0):
    """Steering (rad, + = left) that keeps the car off the walls.
    Centring: a gentle push toward the middle, faded out in bends (where unequal sides are normal).
    Danger: when one wall is closer than danger_m, a push away from it that grows to full as the gap closes."""
    if left is None or right is None or half_corridor <= 0:
        return 0.0
    centre = center_gain * max_steer * (1.0 - curve_frac) * float(np.clip((left - right) / 2.0 / half_corridor, -1.0, 1.0))
    push = 0.0
    near, away = (left, -1.0) if left < right else (right, 1.0)  # a wall on the left pushes right (-), and vice versa
    if near < danger_m:
        push = away * max_steer * float(np.clip((danger_m - near) / danger_m, 0.0, 1.0))
    return float(np.clip(centre + push, -max_steer, max_steer))

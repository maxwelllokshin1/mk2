"""
lap_logic.py: lap counting from LiDAR scans alone (no odometry, no map). Pure numpy, no ROS.

Idea: when the car leaves the start line, remember what the LiDAR saw there (the "reference").
Every scan is squeezed into a few range-per-direction numbers (a signature). A lap is counted when
the signature comes back close to the reference after it was far from it (the car really went
around) and at least min_lap_s have passed.
"""
import math
from collections import deque

import numpy as np


def signature(ranges, angle_min, angle_increment, n_bins=36, max_range=6.0):
    """Mean range per direction around the car (bin 0 starts at -pi). Bins with no usable reading are NaN.
    'No return' (inf) counts as max_range, 0 / NaN readings are dropped. Works for a 360 deg or a 240 deg lidar."""
    r = np.asarray(ranges, dtype=float)
    ang = (angle_min + angle_increment * np.arange(len(r)) + math.pi) % (2 * math.pi)
    idx = np.minimum((ang / (2 * math.pi) * n_bins).astype(int), n_bins - 1)
    r = np.where(np.isposinf(r), max_range, r)
    ok = np.isfinite(r) & (r > 0.05)
    r = np.minimum(r, max_range)
    total = np.bincount(idx[ok], weights=r[ok], minlength=n_bins)
    count = np.bincount(idx[ok], minlength=n_bins)
    with np.errstate(invalid='ignore', divide='ignore'):
        return np.where(count > 0, total / count, np.nan)


def distance(a, b, min_bins=8):
    """Mean |range difference| in metres over the directions both scans saw. inf = too little to compare."""
    both = np.isfinite(a) & np.isfinite(b)
    if both.sum() < min_bins:
        return math.inf
    return float(np.mean(np.abs(a[both] - b[both])))


class LapDetector:
    def __init__(self, match_m=0.6, away_m=1.2, min_lap_s=8.0, min_bins=8, standby_scans=5):
        self.match_m, self.away_m, self.min_lap_s, self.min_bins = match_m, away_m, min_lap_s, min_bins
        self.standby = deque(maxlen=standby_scans)
        self.ref = None
        self.laps = 0
        self.away = False
        self.last_lap_t = 0.0
        self.dist = math.inf          # last distance to the reference (for logging / tuning)

    def standby_scan(self, sig):
        """Feed scans while the car is parked: the last few are averaged into the start-line reference."""
        self.standby.append(sig)

    def arm(self, t):
        """Car leaves the start line: the parked scans become the reference and the lap count restarts."""
        if not self.standby:
            return False
        with np.errstate(all='ignore'):
            self.ref = np.nanmean(np.stack(list(self.standby)), axis=0)
        self.laps, self.away, self.last_lap_t = 0, False, t
        return True

    def disarm(self):
        self.ref = None

    def update(self, sig, t):
        """One scan while driving. True exactly when a lap was just completed."""
        if self.ref is None:
            return False
        self.dist = distance(sig, self.ref, self.min_bins)
        if self.dist > self.away_m:
            self.away = True
        if self.away and self.dist < self.match_m and t - self.last_lap_t >= self.min_lap_s:
            self.laps += 1
            self.last_lap_t, self.away = t, False
            return True
        return False

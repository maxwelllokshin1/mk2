"""
hazard_logic.py: "about to hit a wall" and "hit something" from the LiDAR's view straight ahead. Pure numpy, no ROS.

    front_range()    nearest thing in a narrow cone straight ahead
    HazardDetector   surprise   something ahead is close AND getting closer fast
                     collision  something ahead is almost touching the lidar
                     Driving forward: both count. Parked (while_stationary): a surprise still counts when something
                     rushes at the car, and a collision only if it arrived fast (so parking next to a wall is calm).
                     Each has its own cooldown, so one wall gives one reaction.
"""
import math
from collections import deque

import numpy as np


def front_range(ranges, angle_min, angle_increment, cone_deg=20.0, min_valid=0.05):
    """Nearest valid reading within +-cone_deg of straight ahead (0 rad), or inf. Works for a Hokuyo (0 = ahead)
    and for an RPLIDAR that publishes from the back: angles are wrapped to [-pi, pi] first."""
    r = np.asarray(ranges, dtype=float)
    ang = (angle_min + angle_increment * np.arange(len(r)) + math.pi) % (2 * math.pi) - math.pi
    ok = np.isfinite(r) & (r > min_valid) & (np.abs(ang) <= math.radians(cone_deg))
    return float(r[ok].min()) if ok.any() else math.inf


class HazardDetector:
    def __init__(self, surprise_m=0.9, closing_mps=0.5, collision_m=0.2,
                 surprise_cooldown_s=8.0, collision_cooldown_s=10.0, window_s=0.4, while_stationary=True):
        self.surprise_m, self.closing_mps, self.collision_m = surprise_m, closing_mps, collision_m
        self.surprise_cooldown, self.collision_cooldown, self.window = surprise_cooldown_s, collision_cooldown_s, window_s
        self.while_stationary = while_stationary
        self.history = deque()                  # (t, front range)
        self.last_surprise = self.last_collision = -1e9

    def closing_speed(self, now, front):
        """m/s the front range has been shrinking over the last window (0 if not enough history)."""
        while self.history and now - self.history[0][0] > self.window:
            self.history.popleft()
        if not self.history:
            return 0.0
        t0, r0 = self.history[0]
        return (r0 - front) / (now - t0) if now - t0 > 0.05 else 0.0

    def update(self, now, front, driving_forward):
        """One scan. Returns 'collision', 'surprise' or None."""
        speed = self.closing_speed(now, front)
        self.history.append((now, front))
        if not math.isfinite(front) or not (driving_forward or self.while_stationary):
            return None
        arrived_fast = driving_forward or speed > self.closing_mps
        if front < self.collision_m and arrived_fast and now - self.last_collision >= self.collision_cooldown:
            self.last_collision = self.last_surprise = now            # a crash also covers the surprise
            return 'collision'
        if front < self.surprise_m and speed > self.closing_mps and now - self.last_surprise >= self.surprise_cooldown:
            self.last_surprise = now
            return 'surprise'
        return None

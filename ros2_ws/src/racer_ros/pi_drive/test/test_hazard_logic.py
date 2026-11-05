import math

import numpy as np

from pi_drive.hazard_logic import HazardDetector, front_range


def scan(front=3.0, side=3.0, n=360, half_deg=25):
    """A 360-degree scan (1 deg per ray, angle_min = -pi) with 'front' within +-half_deg of straight ahead."""
    ang = -math.pi + np.arange(n) * math.radians(1.0)
    r = np.full(n, side)
    r[np.abs(ang) <= math.radians(half_deg)] = front
    return r.tolist(), -math.pi, math.radians(1.0)


def test_front_range_sees_only_straight_ahead():
    assert abs(front_range(*scan(front=0.5, side=0.1)) - 0.5) < 1e-9
    assert front_range([0.0, float('nan'), float('inf')], -0.1, 0.1) == math.inf


def test_front_range_rplidar_from_the_back_with_negative_increment():
    r, _, inc = scan(front=0.7)
    assert abs(front_range(r[::-1], math.pi - math.radians(1.0), -inc) - 0.7) < 1e-9


def test_wall_ahead_at_speed_is_a_surprise_once():
    det = HazardDetector()
    out = [det.update(i * 0.05, 3.0 - i * 0.1, True) for i in range(25)]      # closing at 2 m/s, stops at 0.6 m
    assert out.count('surprise') == 1 and 'collision' not in out


def test_hitting_it_is_a_collision_and_one_crash_is_one_reaction():
    det = HazardDetector()
    out = [det.update(i * 0.05, max(2.0 - i * 0.1, 0.1), True) for i in range(40)]
    assert out.count('collision') == 1 and out.count('surprise') <= 1


def test_nothing_happens_when_not_driving_forward_if_stationary_is_off():
    det = HazardDetector(while_stationary=False)
    assert not any(det.update(i * 0.05, max(2.0 - i * 0.1, 0.1), False) for i in range(40))


def test_parked_car_is_surprised_when_something_rushes_at_it():
    det = HazardDetector()
    out = [det.update(i * 0.05, 3.0 - i * 0.1, False) for i in range(25)]      # a wall / person closing at 2 m/s
    assert out.count('surprise') == 1


def test_parked_next_to_a_wall_is_calm():
    det = HazardDetector()
    assert not any(det.update(i * 0.05, 0.15, False) for i in range(40))       # 0.15 m from the lidar, not moving


def test_parked_close_to_a_wall_or_crawling_is_no_surprise():
    det = HazardDetector()
    assert not any(det.update(i * 0.05, 0.8, True) for i in range(40))        # 0.8 m away, not closing
    det = HazardDetector()
    assert not any(det.update(i * 0.05, 0.85 - i * 0.002, True) for i in range(40))   # 0.04 m/s: crawling

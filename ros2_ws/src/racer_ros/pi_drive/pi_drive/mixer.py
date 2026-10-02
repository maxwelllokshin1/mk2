"""
mixer.py: pure control logic (no ROS, no hardware), so it can be unit tested.

Turns "what the Joy-Cons are doing" + "what the autonomy node last said" into
one command: (steering, throttle, mode).

    steering   -1..1, + = LEFT (same sign convention as Ackermann steering_angle)
    throttle   -1..1, + = forward, - = reverse
    mode       NO_CONTROLLER | RC | AUTO | AUTO_WAITING

Rules
    R held            -> AUTO: steering from the autonomy node's /drive, throttle
                         scales with the autonomy node's speed up to auto_max_throttle
                         (or a fixed value if auto_speed_ref_mps <= 0); reverse follows the sign. Release R -> back to RC immediately.
    R not held (RC)   -> steering from the stick, A = forward, B = reverse,
                         A and B together = stop.
    nothing connected -> everything 0.
"""
import math
from dataclasses import dataclass


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def apply_deadzone(v, dz):
    """Zero inside the deadzone, rescaled so output still reaches +-1."""
    if abs(v) <= dz:
        return 0.0
    return math.copysign((abs(v) - dz) / (1.0 - dz), v)


@dataclass
class MixerConfig:
    steer_deadzone: float = 0.12
    steer_sign: float = -1.0           # raw stick axis -> + = left. Flip if steering is backwards
    rc_forward: float = 0.30           # throttle while A is held
    rc_reverse: float = 0.25           # throttle (magnitude) while B is held
    auto_throttle: float = 0.25        # fixed autonomy throttle; only used when auto_speed_ref_mps <= 0
    auto_speed_ref_mps: float = 15.0   # reactive_node speed that maps to auto_max_throttle. Set = gap_params speed_max (>0 = follow its speed)
    auto_max_throttle: float = 0.25    # THE autonomy speed ceiling: raise as the car proves itself
    auto_min_throttle: float = 0.12    # floor while moving, so slow sections still get past the ESC deadband
    auto_reverse_throttle: float = 0.25
    auto_max_steer_rad: float = 0.52   # autonomy steering_angle that equals full lock (reactive_node max_steering = 30 deg)
    auto_speed_deadband: float = 0.05  # |autonomy speed| below this = stop
    auto_timeout_s: float = 0.5        # /drive older than this is treated as missing


@dataclass
class Inputs:
    have_steer: bool = False   # the Joy-Con with the steering stick is connected
    have_buttons: bool = False # the Joy-Con with A / B / R is connected
    stick: float = 0.0         # raw steering axis, -1..1
    a: bool = False
    b: bool = False
    r: bool = False


class Mixer:
    def __init__(self, cfg: MixerConfig):
        self.cfg = cfg

    def mix(self, inp: Inputs, auto_steer_rad: float = 0.0,
            auto_speed: float = 0.0, auto_age_s: float = float('inf')):
        c = self.cfg

        if not inp.have_steer and not inp.have_buttons:
            return 0.0, 0.0, 'NO_CONTROLLER'

        if inp.have_buttons and inp.r:
            if auto_age_s > c.auto_timeout_s:
                return 0.0, 0.0, 'AUTO_WAITING'
            steering = clamp(auto_steer_rad / c.auto_max_steer_rad, -1.0, 1.0)
            if auto_speed > c.auto_speed_deadband:
                if c.auto_speed_ref_mps > 0.0:
                    # follow reactive_node's own speed (it already slows for tight tracks / corners)
                    frac = clamp(auto_speed / c.auto_speed_ref_mps, 0.0, 1.0)
                    throttle = max(c.auto_min_throttle, frac * c.auto_max_throttle)
                    throttle = min(throttle, c.auto_max_throttle)
                else:
                    throttle = c.auto_throttle
            elif auto_speed < -c.auto_speed_deadband:
                throttle = -c.auto_reverse_throttle
            else:
                throttle = 0.0
            return steering, throttle, 'AUTO'

        steering = 0.0
        if inp.have_steer:
            steering = clamp(c.steer_sign * apply_deadzone(inp.stick, c.steer_deadzone), -1.0, 1.0) + 0.0  # + 0.0 turns -0.0 into 0.0

        throttle = 0.0
        if inp.have_buttons:
            if inp.a and not inp.b:
                throttle = c.rc_forward
            elif inp.b and not inp.a:
                throttle = -c.rc_reverse
        return steering, throttle, 'RC'

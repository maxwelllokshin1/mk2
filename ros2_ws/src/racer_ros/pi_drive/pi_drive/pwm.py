"""
pwm.py: command -> pulse width, throttle shaping, and the GPIO output classes.

Hobby servos and ESCs both take a 50 Hz pulse whose HIGH time is the command:
~1500 us = center / neutral, shorter/longer = either side.
"""
import time
from dataclasses import dataclass

from pi_drive.mixer import clamp


@dataclass
class PulseMap:
    # steering servo
    steer_center_us: float = 1500.0
    steer_range_us: float = 200.0      # us from center at full lock (steering = +-1)
    steer_sign: float = 1.0            # +1: steering + (left) -> longer pulse. Flip if it turns the wrong way
    steer_min_us: float = 1300.0       # hard clamps, protect the linkage
    steer_max_us: float = 1700.0
    # ESC
    throttle_neutral_us: float = 1500.0
    throttle_deadband_us: float = 25.0 # ESCs ignore a few us around neutral; skip over it so small throttle actually moves
    throttle_fwd_max_us: float = 1700.0
    throttle_rev_min_us: float = 1300.0

    def steering_to_us(self, steering: float) -> float:
        us = self.steer_center_us + self.steer_sign * clamp(steering, -1.0, 1.0) * self.steer_range_us
        lo, hi = sorted((self.steer_min_us, self.steer_max_us))
        return clamp(us, lo, hi)

    def throttle_to_us(self, throttle: float) -> float:
        t = clamp(throttle, -1.0, 1.0)
        n = self.throttle_neutral_us
        if t > 0.0:
            start = n + self.throttle_deadband_us
            return clamp(start + t * (self.throttle_fwd_max_us - start), n, self.throttle_fwd_max_us)
        if t < 0.0:
            start = n - self.throttle_deadband_us
            return clamp(start + t * (start - self.throttle_rev_min_us), self.throttle_rev_min_us, n)
        return n


class ThrottleShaper:
    """
    Makes throttle safe for the ESC:
      - speeding up is rate limited (no 0 -> full jerk); letting off is instant.
      - going forward <-> reverse always passes through neutral for
        `reverse_neutral_s` (many car ESCs ignore a direct jump to reverse).
    """
    def __init__(self, slew_per_s=2.0, reverse_neutral_s=0.25):
        self.slew_per_s = slew_per_s
        self.reverse_neutral_s = reverse_neutral_s
        self.value = 0.0
        self._last_dir = 0
        self._neutral_time = 0.0

    @staticmethod
    def _sign(x):
        return (x > 0) - (x < 0)

    def step(self, target: float, dt: float) -> float:
        if self.value == 0.0:
            self._neutral_time += dt
        else:
            self._neutral_time = 0.0

        want = self._sign(target)
        if want != 0 and self._last_dir != 0 and want != self._last_dir:
            # direction change: drop to neutral, hold there before reversing
            if self.value != 0.0 or self._neutral_time < self.reverse_neutral_s:
                self.value = 0.0
                return 0.0

        if abs(target) > abs(self.value):
            max_step = self.slew_per_s * dt
            self.value = clamp(target, self.value - max_step, self.value + max_step)
        else:
            self.value = target

        if self.value != 0.0:
            self._last_dir = self._sign(self.value)
        return self.value


class DryOutput:
    """No hardware: remembers what it was told. Used when no pins are configured."""
    live = False

    def __init__(self):
        self.pulses = {}

    def open(self, pins):
        self.pulses = {p: 0.0 for p in pins}

    def set_us(self, pin, us):
        self.pulses[pin] = us

    def close(self):
        pass


class LgpioOutput:
    """
    Real pulses via lgpio's tx_servo (software timed, 50 Hz). Works on any GPIO
    pin; expect a little jitter when the CPU is busy. Pulses stop if this
    process dies, which is the safe failure for both a servo and an ESC.
    """
    live = True

    def __init__(self, chip=0):
        self.chip = chip
        self._lg = None
        self._h = None
        self._pins = []
        self._last = {}

    def open(self, pins):
        import lgpio
        self._lg = lgpio
        self._h = lgpio.gpiochip_open(self.chip)
        for p in pins:
            lgpio.gpio_claim_output(self._h, p)
            self._pins.append(p)

    def set_us(self, pin, us):
        us = int(round(us))
        if self._last.get(pin) == us:   # restarting the pulse train on every tick makes the servo jitter
            return
        self._lg.tx_servo(self._h, pin, us, 50)
        self._last[pin] = us

    def close(self):
        if self._h is None:
            return
        for p in self._pins:
            try:
                self._lg.tx_servo(self._h, p, 0)   # width 0 = stop pulses
            except Exception:
                pass
        try:
            self._lg.gpiochip_close(self._h)
        except Exception:
            pass
        self._h = None

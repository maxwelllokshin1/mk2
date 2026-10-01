"""
backends/gpio_backend.py — NO microcontroller: the Pi makes the servo/ESC pulses
itself from a GPIO pin.

WHY IT EXISTS
    It answers "can I test steering with just the Pi?" (yes). It needs no
    firmware, no protocol, no serial. Useful for the first day with a servo.

WHY YOU SHOULDN'T DRIVE ON IT
    - Linux is not a real-time OS. lgpio's tx_servo makes SOFTWARE-timed pulses,
      so when the CPU is busy (lidar + Python nodes on a Pi 3) the pulse width
      jitters and the servo twitches. Hardware/DMA PWM (pigpio, or the Pi's
      PWM0/PWM1 pins) is steadier.
    - There is NO watchdog below bridge_node. On the Arduino, if the Pi dies the
      board notices and centers the wheels. Here, if the process is killed
      hard, the last pulse may keep running. Test that with the wheels off the ground.
    - No feedback, so poll() returns None (inherited) -> use odom_mode: commanded.

HOW IT WORKS (lgpio is already installed by the Dockerfile)
    Servo signal = a 50 Hz pulse train whose HIGH time (1000-2000 us, 1500 =
    center) is the command. Same physics as mcu_ws/src/steering.cpp, and the same
    math (read that file's comment, the calibration procedure is identical):
        us = center + sign * degrees(steering_rad) * us_per_deg
        us = clamp(us, min, max)              <- the safety clamp, keep it
    lgpio calls:
        h = lgpio.gpiochip_open(0)                    Pi 3: chip 0
        lgpio.gpio_claim_output(h, gpio)              take ownership of the pin
        lgpio.tx_servo(h, gpio, pulse_us, 50)         start/update the pulse train (50 Hz)
        lgpio.tx_servo(h, gpio, 0)                    pulse width 0 = stop pulses
        lgpio.gpiochip_close(h)
    Only call tx_servo when the pulse width CHANGED (remember the last value per
    pin), or you keep restarting the pulse train.
    Pins: GPIO18 is header pin 12. The Pi's GPIO is 3.3 V; most servos accept a
    3.3 V signal. Power the servo from a BEC, never from the Pi's 5 V pin, and
    connect the grounds.

`cal` is a dict with: steer_gpio, throttle_gpio (-1 = none), steer_center_us,
steer_us_per_deg, steer_min_us, steer_max_us, steer_sign, throttle_neutral_us,
throttle_us_per_mps, throttle_min_us, throttle_max_us   (from bridge_params.yaml).
"""
from mcu_bridge.backends.base import Backend


class GpioBackend(Backend):
    def __init__(self, cal: dict):
        # TODO: store cal; self._h = None; self._lg = None; self._last_us = {}
        raise NotImplementedError

    def open(self) -> None:
        # TODO: import lgpio (inside this method), open the chip, claim the
        #       steering pin (and the throttle pin if throttle_gpio >= 0), then
        #       send(0.0, 0.0) so the wheels start straight.
        raise NotImplementedError

    def send(self, steering_rad: float, speed_mps: float) -> None:
        # TODO: steering us (with clamp) -> tx_servo. Throttle the same way
        #       (neutral + speed * us_per_mps, clamped) only if throttle_gpio >= 0.
        raise NotImplementedError

    def close(self) -> None:
        # TODO: send(0.0, 0.0), a short sleep so it takes effect, stop pulses on
        #       each pin you used (tx_servo width 0), close the chip. Handle
        #       "never opened" (self._h is None) without raising.
        raise NotImplementedError

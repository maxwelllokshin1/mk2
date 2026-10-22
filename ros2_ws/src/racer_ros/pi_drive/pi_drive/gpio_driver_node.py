#!/usr/bin/env python3
import math
import os
import signal
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32MultiArray, String

# Pi 5 hardware PWM (dtoverlay=pwm-2chan): which pwmchip channel drives which BCM pin
PIN_TO_CHANNEL = {12: 0, 13: 1, 18: 2, 19: 3}


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def sign(x):
    return (x > 0) - (x < 0)


class HwPwm:
    """Hardware PWM through sysfs (Pi 5: GPIO18 = pwmchip0 channel 2, GPIO19 = channel 3,
    needs dtoverlay=pwm-2chan). The RP1 chip generates the pulses, so CPU load cannot make them jitter."""

    def __init__(self, chip, channel, hz, start_us):
        self.base = f'/sys/class/pwm/pwmchip{chip}'
        self.ch = f'{self.base}/pwm{channel}'
        if not os.path.isdir(self.ch):
            self._write(f'{self.base}/export', channel)
            for _ in range(50):
                if os.path.isdir(self.ch):
                    break
                time.sleep(0.02)
        self._write(f'{self.ch}/duty_cycle', 0)   # duty must stay <= period while changing it
        self._write(f'{self.ch}/period', int(1e9 / hz))
        self.set_us(start_us)
        self._write(f'{self.ch}/enable', 1)

    @staticmethod
    def _write(path, value):
        for _ in range(50):   # udev may still be fixing permissions right after export
            try:
                with open(path, 'w') as f:
                    f.write(str(int(value)))
                return
            except PermissionError:
                time.sleep(0.04)
        with open(path, 'w') as f:
            f.write(str(int(value)))

    def set_us(self, us):
        self._write(f'{self.ch}/duty_cycle', us * 1000)

    def stop(self):
        self._write(f'{self.ch}/enable', 0)


class SoftPwm:
    """Fallback for steering: lgpio software timing through the RPi.GPIO shim (jitters when the CPU is busy)."""

    def __init__(self, pin, hz, start_us):
        import RPi.GPIO as GPIO
        self.GPIO = GPIO
        self.hz = hz
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT)
        self.pwm = GPIO.PWM(pin, hz)
        self.pwm.start(self._duty(start_us))

    def _duty(self, us):
        return us * self.hz / 1e6 * 100.0

    def set_us(self, us):
        self.pwm.ChangeDutyCycle(self._duty(us))

    def stop(self):
        self.pwm.stop()
        self.pwm = None          # drop it BEFORE cleanup(), or PWM.__del__ hits a closed chip
        self.GPIO.cleanup()


class GpioDriver(Node):
    def __init__(self):
        super().__init__('gpio_driver')

        # ---- parameters (names match pi_drive.yaml; floats MUST be floats there) ----
        self.declare_parameter('steer_gpio', -1)             # BCM number, -1 = not connected
        self.declare_parameter('throttle_gpio', -1)          # BCM number (12/13/18/19 = hardware PWM), -1 = off
        self.declare_parameter('gpiochip', 0)                # unused
        self.declare_parameter('pwm_backend', 'auto')        # auto | hw | soft  (steering only)
        self.declare_parameter('pwm_chip', 0)                # Pi 5 kernel 6.18: pwmchip0 (older: 2)
        self.declare_parameter('pwm_channel', 2)             # steering: Pi 5 channel 2 = GPIO18, 3 = GPIO19
        self.declare_parameter('pwm_hz', 50.0)
        self.declare_parameter('command_timeout_s', 0.5)
        self.declare_parameter('arm_time_s', 3.0)
        self.declare_parameter('throttle_slew_per_s', 2.0)
        self.declare_parameter('reverse_neutral_s', 0.25)
        self.declare_parameter('steer_center_us', 1500.0)
        self.declare_parameter('steer_range_us', 200.0)
        self.declare_parameter('steer_sign', 1.0)
        self.declare_parameter('steer_min_us', 1300.0)
        self.declare_parameter('steer_max_us', 1700.0)
        self.declare_parameter('recenter_overshoot_left_us', 40.0)    # kick after a LEFT turn (0 = off)
        self.declare_parameter('recenter_overshoot_right_us', 80.0)   # kick after a RIGHT turn (0 = off)
        self.declare_parameter('recenter_overshoot_s', 0.15)
        self.declare_parameter('recenter_min_deflection_us', 50.0)
        self.declare_parameter('throttle_neutral_us', 1500.0)
        self.declare_parameter('throttle_deadband_us', 25.0)
        self.declare_parameter('throttle_fwd_max_us', 1700.0)
        self.declare_parameter('throttle_rev_min_us', 1300.0)
        self.declare_parameter('odom_speed_at_full_throttle', 2.0)
        self.declare_parameter('odom_max_steer_rad', 0.52)
        self.declare_parameter('wheelbase', 0.25)

        p = self.get_parameter
        self.steer_pin = int(p('steer_gpio').value)
        self.throttle_pin = int(p('throttle_gpio').value)
        self.pwm_hz = float(p('pwm_hz').value)
        self.pwm_backend = str(p('pwm_backend').value)
        self.pwm_chip = int(p('pwm_chip').value)
        self.pwm_channel = int(p('pwm_channel').value)
        self.timeout = float(p('command_timeout_s').value)
        self.arm_time = float(p('arm_time_s').value)
        self.slew = float(p('throttle_slew_per_s').value)
        self.reverse_neutral = float(p('reverse_neutral_s').value)
        self.steer_center = float(p('steer_center_us').value)
        self.steer_range = float(p('steer_range_us').value)
        self.steer_sign = float(p('steer_sign').value)
        self.steer_min = float(p('steer_min_us').value)
        self.steer_max = float(p('steer_max_us').value)
        self.overshoot_left = float(p('recenter_overshoot_left_us').value)
        self.overshoot_right = float(p('recenter_overshoot_right_us').value)
        self.overshoot_s = float(p('recenter_overshoot_s').value)
        self.overshoot_min = float(p('recenter_min_deflection_us').value)
        self.thr_neutral = float(p('throttle_neutral_us').value)
        self.thr_deadband = float(p('throttle_deadband_us').value)
        self.thr_fwd_max = float(p('throttle_fwd_max_us').value)
        self.thr_rev_min = float(p('throttle_rev_min_us').value)
        self.v_full = float(p('odom_speed_at_full_throttle').value)
        self.max_steer = float(p('odom_max_steer_rad').value)
        self.wheelbase = float(p('wheelbase').value)

        # ---- state ----
        self.cmd_steer = 0.0
        self.cmd_throttle = 0.0
        self.last_cmd_time = None
        self.timed_out = True
        self.steer_applied = 0.0
        self.throttle_applied = 0.0
        self.last_dir = 0                    # sign of the last non-zero throttle (for fwd<->rev pause)
        self.neutral_since = self._now_s()
        self.last_us = None
        self.peak_off = 0.0                  # biggest steering offset (us) since the wheel was last centered
        self.kick_until = 0.0                # recentering overshoot is active until this time
        self.kick_us = None
        self.last_thr_us = None
        self.armed = False
        self.controller = False              # a Joy-Con is connected (teleop publishes /drive_mode)
        self.arm_until = float('inf')        # ESC arming starts when the controller connects, not at boot
        self.last_tick = self._now_s()
        self.x = self.y = self.yaw = 0.0
        self.closed = False

        # ---- pin setup ----
        self.dry = self.steer_pin < 0
        self.pwm = None
        self.thr_pwm = None
        if self.dry:
            self.get_logger().warn('steer_gpio is -1: DRY RUN, steering will not move')
        else:
            self.pwm = self._open_pwm()
        if self.throttle_pin >= 0:
            self.thr_pwm = self._open_throttle()
        else:
            self.get_logger().warn('throttle_gpio is -1: throttle off, motor will not move')

        self.add_on_set_parameters_callback(self._on_params)   # lets the dashboard Tuning tab change these live

        # ---- ROS io ----
        self.create_subscription(Float32MultiArray, '/drive_cmd', self._on_cmd, 10)
        self.create_subscription(String, '/drive_mode', self._on_mode, 10)
        self.odom_pub = self.create_publisher(Odometry, '/ego_racecar/odom', 10)
        self.create_timer(0.02, self._tick)          # 50 Hz control loop
        self.create_timer(0.05, self._publish_odom)  # 20 Hz odom

        self.get_logger().info(
            f'GPIO DRIVER ON: steer_gpio={self.steer_pin} pwm={type(self.pwm).__name__ if self.pwm else "none"} '
            f'(center {self.steer_center:.0f}us, +-{self.steer_range:.0f}us, '
            f'clamp {self.steer_min:.0f}-{self.steer_max:.0f}us) | '
            f'throttle_gpio={self.throttle_pin} {"ON" if self.thr_pwm else "off"} '
            f'(neutral {self.thr_neutral:.0f}us, fwd<={self.thr_fwd_max:.0f}, rev>={self.thr_rev_min:.0f}), '
            f'idle until a controller connects, then arming ESC for {self.arm_time:.1f}s')

    # ---------------- helpers ----------------
    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _open_pwm(self):
        """Steering: hardware PWM when the sysfs channel is there, else software (backend 'auto')."""
        if self.pwm_backend in ('auto', 'hw'):
            try:
                return HwPwm(self.pwm_chip, self.pwm_channel, self.pwm_hz, self.steer_center)
            except OSError as e:
                if self.pwm_backend == 'hw':
                    raise
                self.get_logger().warn(
                    f'hardware PWM unavailable ({e}); using software PWM (will jitter)')
        return SoftPwm(self.steer_pin, self.pwm_hz, self.steer_center)

    def _open_throttle(self):
        """Throttle: hardware PWM only (the overlay owns GPIO 12/13/18/19). If it fails the motor stays off."""
        channel = PIN_TO_CHANNEL.get(self.throttle_pin)
        if channel is None:
            self.get_logger().error(
                f'throttle_gpio={self.throttle_pin} has no hardware PWM (use 12, 13, 18 or 19): motor off')
            return None
        try:
            return HwPwm(self.pwm_chip, channel, self.pwm_hz, self.thr_neutral)
        except OSError as e:
            self.get_logger().error(f'throttle hardware PWM failed ({e}): motor off')
            return None

    def _steering_to_us(self, steering):
        us = self.steer_center + self.steer_sign * steering * self.steer_range
        return clamp(us, self.steer_min, self.steer_max)

    def _throttle_to_us(self, t):
        if t == 0.0:
            return self.thr_neutral
        if t > 0:   # skip the ESC deadband so the smallest command already moves
            lo = self.thr_neutral + self.thr_deadband
            return clamp(lo + t * (self.thr_fwd_max - lo), lo, self.thr_fwd_max)
        hi = self.thr_neutral - self.thr_deadband
        return clamp(hi + t * (hi - self.thr_rev_min), self.thr_rev_min, hi)

    def _apply_steering(self, steering, now):
        us = self._steering_to_us(steering)
        off = us - self.steer_center
        centered = abs(off) <= 10.0

        if centered:
            # Back to center after a real turn: go slightly PAST center first (the linkage has slack and
            # otherwise stops short), hold briefly, then settle on center.
            # steering > 0 is a LEFT turn (same convention as the rest of the car), whatever steer_sign is
            max_us = self.overshoot_left if self.peak_off * self.steer_sign > 0 else self.overshoot_right
            if (self.kick_until == 0.0 and max_us > 0.0 and abs(self.peak_off) >= self.overshoot_min):
                amount = max_us * min(1.0, abs(self.peak_off) / (0.5 * self.steer_range))
                self.kick_us = clamp(self.steer_center - math.copysign(amount, self.peak_off),
                                     self.steer_min, self.steer_max)
                self.kick_until = now + self.overshoot_s
            self.peak_off = 0.0
            if self.kick_until > 0.0:
                if now < self.kick_until:
                    us = self.kick_us
                else:
                    self.kick_until = 0.0          # kick done: settle on center
        else:
            self.kick_until = 0.0                  # moved away from center: no kick
            if self.peak_off * off < 0.0:
                self.peak_off = 0.0                # went through center to the other side
            if abs(off) > abs(self.peak_off):
                self.peak_off = off

        if self.last_us is not None and abs(us - self.last_us) < 2.0:
            return  # stick noise, don't touch the PWM
        self.last_us = us
        if self.dry:
            self.get_logger().info(f'[dry] steer={us:.0f}us', throttle_duration_sec=1.0)
        else:
            self.pwm.set_us(us)

    def _update_throttle(self, target, stale, now, dt):
        """Safe throttle: neutral while arming / stale, ramped up, paused at neutral between forward and reverse."""
        cur = self.throttle_applied
        if not self.controller:
            self.armed = False
            stale = True                                     # no controller: hold neutral
        elif not self.armed:
            if now >= self.arm_until:
                self.armed = True
                self.get_logger().info('ESC armed: throttle enabled')
            else:
                target = 0.0
        if stale:
            new = 0.0                                        # lost commands: neutral NOW
        else:
            if target * cur < 0:
                target = 0.0                                 # reversing direction: pass through neutral first
            if (target != 0.0 and cur == 0.0 and self.last_dir != 0 and sign(target) != self.last_dir
                    and now - self.neutral_since < self.reverse_neutral):
                target = 0.0                                 # wait at neutral before the other direction
            if abs(target) < abs(cur):
                new = target                                 # easing off is instant
            else:
                step = self.slew * dt                        # speeding up is ramped
                new = cur + clamp(target - cur, -step, step)
        if new == 0.0 and cur != 0.0:
            self.neutral_since = now
            self.last_dir = sign(cur)
        elif new != 0.0:
            self.last_dir = sign(new)
        self.throttle_applied = new

        us = self._throttle_to_us(new)
        if us != self.last_thr_us:
            self.last_thr_us = us
            if self.thr_pwm is not None:
                self.thr_pwm.set_us(us)

    # ---------------- callbacks ----------------
    def _on_cmd(self, msg):
        if len(msg.data) < 2:
            return
        self.cmd_steer = clamp(float(msg.data[0]), -1.0, 1.0)
        self.cmd_throttle = clamp(float(msg.data[1]), -1.0, 1.0)
        self.last_cmd_time = self.get_clock().now()

    # parameter -> (attribute it sets, lowest, highest). The dashboard may change these while the car runs.
    LIVE = {'throttle_slew_per_s': ('slew', 0.02, 5.0), 'steer_center_us': ('steer_center', 1000.0, 2000.0),
            'steer_range_us': ('steer_range', 0.0, 500.0), 'recenter_overshoot_left_us': ('overshoot_left', 0.0, 200.0),
            'recenter_overshoot_right_us': ('overshoot_right', 0.0, 200.0), 'recenter_overshoot_s': ('overshoot_s', 0.0, 1.0)}

    def _on_params(self, params):
        for p in params:
            if p.name not in self.LIVE:
                continue
            _, lo, hi = self.LIVE[p.name]
            try:
                v = float(p.value)
            except (TypeError, ValueError):
                return SetParametersResult(successful=False, reason=f'{p.name} must be a number')
            if not lo <= v <= hi:
                return SetParametersResult(successful=False, reason=f'{p.name} must be between {lo:g} and {hi:g}')
        for p in params:
            if p.name in self.LIVE:
                setattr(self, self.LIVE[p.name][0], float(p.value))
                self.get_logger().info(f'{p.name} changed to {float(p.value):g}')
                if p.name.startswith('steer_'):
                    self.last_us = None          # send the steering pulse again with the new numbers
        return SetParametersResult(successful=True)

    def _on_mode(self, msg):
        connected = msg.data != 'NO_CONTROLLER'
        if connected and not self.controller:
            self.controller = True
            self.armed = False
            self.arm_until = self._now_s() + self.arm_time
            self.get_logger().info(f'controller connected: arming ESC for {self.arm_time:.1f}s')
        elif not connected and self.controller:
            self.controller = False
            self.armed = False
            self.arm_until = float('inf')
            self.get_logger().warn('controller lost: motor neutral, waiting for a controller')

    def _tick(self):
        now = self._now_s()
        dt = max(1e-3, min(0.1, now - self.last_tick))
        self.last_tick = now
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9 if self.last_cmd_time else float('inf')
        stale = age > self.timeout

        steering = 0.0 if stale else self.cmd_steer
        throttle = 0.0 if stale else self.cmd_throttle

        if stale != self.timed_out:
            self.timed_out = stale
            if stale:
                self.get_logger().warn('no /drive_cmd: steering centered, throttle neutral')
            else:
                self.get_logger().info('/drive_cmd received')

        self.steer_applied = steering
        self._apply_steering(steering, now)
        self._update_throttle(throttle, stale, now, dt)

    def _publish_odom(self):
        dt = 0.05
        v = self.throttle_applied * self.v_full   # dead reckoned from the throttle command
        delta = self.steer_applied * self.max_steer
        self.yaw += v * math.tan(delta) / self.wheelbase * dt
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_link'
        m.pose.pose.position.x = self.x
        m.pose.pose.position.y = self.y
        m.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        m.twist.twist.linear.x = v
        self.odom_pub.publish(m)

    def safe_stop(self):
        """Throttle to neutral FIRST, center the steering, let the pulses land, then stop PWM."""
        if self.closed:
            return
        self.closed = True
        if self.thr_pwm is not None:
            self.thr_pwm.set_us(self.thr_neutral)
        if self.pwm is not None:
            self.pwm.set_us(self.steer_center)
        time.sleep(0.3)
        if self.thr_pwm is not None:
            self.thr_pwm.stop()
        if self.pwm is not None:
            self.pwm.stop()


def main(args=None):
    rclpy.init(args=args)
    node = GpioDriver()
    # docker stop sends SIGTERM; make it end spin() like Ctrl+C does
    signal.signal(signal.SIGTERM, lambda *_: rclpy.try_shutdown())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.safe_stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

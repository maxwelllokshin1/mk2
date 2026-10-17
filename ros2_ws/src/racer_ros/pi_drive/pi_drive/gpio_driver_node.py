#!/usr/bin/env python3
import math
import os
import signal
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class HwPwm:
    """Hardware PWM through sysfs (Pi 5: GPIO18 = pwmchip2 channel 2, needs dtoverlay=pwm-2chan).
    The RP1 chip generates the pulses, so CPU load cannot make them jitter."""

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
    """Fallback: lgpio software timing through the RPi.GPIO shim (jitters when the CPU is busy)."""

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
        self.declare_parameter('throttle_gpio', -1)          # unused for now
        self.declare_parameter('gpiochip', 0)                # unused
        self.declare_parameter('pwm_backend', 'auto')        # auto | hw | soft
        self.declare_parameter('pwm_chip', 2)                # Pi 5: pwmchip2
        self.declare_parameter('pwm_channel', 2)             # Pi 5: channel 2 = GPIO18, 3 = GPIO19
        self.declare_parameter('pwm_hz', 50.0)
        self.declare_parameter('command_timeout_s', 0.5)
        self.declare_parameter('steer_center_us', 1500.0)
        self.declare_parameter('steer_range_us', 200.0)
        self.declare_parameter('steer_sign', 1.0)
        self.declare_parameter('steer_min_us', 1300.0)
        self.declare_parameter('steer_max_us', 1700.0)
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
        self.steer_center = float(p('steer_center_us').value)
        self.steer_range = float(p('steer_range_us').value)
        self.steer_sign = float(p('steer_sign').value)
        self.steer_min = float(p('steer_min_us').value)
        self.steer_max = float(p('steer_max_us').value)
        self.v_full = float(p('odom_speed_at_full_throttle').value)
        self.max_steer = float(p('odom_max_steer_rad').value)
        self.wheelbase = float(p('wheelbase').value)

        # ---- state ----
        self.cmd_steer = 0.0
        self.cmd_throttle = 0.0
        self.last_cmd_time = None
        self.timed_out = True
        self.steer_applied = 0.0
        self.last_us = None
        self.x = self.y = self.yaw = 0.0
        self.closed = False

        # ---- pin setup ----
        self.dry = self.steer_pin < 0
        self.pwm = None
        if self.dry:
            self.get_logger().warn('steer_gpio is -1: DRY RUN, nothing will move')
        else:
            self.pwm = self._open_pwm()

        # ---- ROS io ----
        self.create_subscription(Float32MultiArray, '/drive_cmd', self._on_cmd, 10)
        self.odom_pub = self.create_publisher(Odometry, '/ego_racecar/odom', 10)
        self.create_timer(0.02, self._tick)          # 50 Hz control loop
        self.create_timer(0.05, self._publish_odom)  # 20 Hz odom

        self.get_logger().info(
            f'GPIO DRIVER ON: steer_gpio={self.steer_pin} pwm={type(self.pwm).__name__ if self.pwm else "none"} '
            f'(center {self.steer_center:.0f}us, +-{self.steer_range:.0f}us, '
            f'clamp {self.steer_min:.0f}-{self.steer_max:.0f}us)')

    # ---------------- helpers ----------------
    def _open_pwm(self):
        """Hardware PWM when the sysfs channel is there, else software (backend 'auto')."""
        if self.pwm_backend in ('auto', 'hw'):
            try:
                return HwPwm(self.pwm_chip, self.pwm_channel, self.pwm_hz, self.steer_center)
            except OSError as e:
                if self.pwm_backend == 'hw':
                    raise
                self.get_logger().warn(
                    f'hardware PWM unavailable ({e}); using software PWM (will jitter)')
        return SoftPwm(self.steer_pin, self.pwm_hz, self.steer_center)

    def _steering_to_us(self, steering):
        us = self.steer_center + self.steer_sign * steering * self.steer_range
        return clamp(us, self.steer_min, self.steer_max)

    def _apply_steering(self, steering):
        us = self._steering_to_us(steering)
        if self.last_us is not None and abs(us - self.last_us) < 2.0:
            return  # stick noise, don't touch the PWM
        self.last_us = us
        if self.dry:
            self.get_logger().info(f'[dry] steer={us:.0f}us', throttle_duration_sec=1.0)
        else:
            self.pwm.set_us(us)

    # ---------------- callbacks ----------------
    def _on_cmd(self, msg):
        if len(msg.data) < 2:
            return
        self.cmd_steer = clamp(float(msg.data[0]), -1.0, 1.0)
        self.cmd_throttle = clamp(float(msg.data[1]), -1.0, 1.0)
        self.last_cmd_time = self.get_clock().now()

    def _tick(self):
        now = self.get_clock().now()
        age = (now - self.last_cmd_time).nanoseconds / 1e9 if self.last_cmd_time else float('inf')
        stale = age > self.timeout

        steering = 0.0 if stale else self.cmd_steer

        if stale != self.timed_out:
            self.timed_out = stale
            if stale:
                self.get_logger().warn('no /drive_cmd: steering centered')
            else:
                self.get_logger().info('/drive_cmd received')

        self.steer_applied = steering
        self._apply_steering(steering)

    def _publish_odom(self):
        dt = 0.05
        v = 0.0  # no throttle driven yet, so no speed
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
        """Center, let the pulse land, then stop PWM and release the pin."""
        if self.closed:
            return
        self.closed = True
        if self.pwm is not None:
            self.pwm.set_us(self.steer_center)
            time.sleep(0.3)
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
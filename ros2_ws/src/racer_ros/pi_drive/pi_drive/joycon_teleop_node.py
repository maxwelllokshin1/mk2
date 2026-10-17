#!/usr/bin/env python3
"""
joycon_teleop_node: Joy-Cons (read straight from Linux evdev) -> /drive_cmd

    stick, one axis      steering (steer_axis)
    stick, other axis    drive: up = forward, down = reverse, proportional (throttle_axis).
                         With throttle_axis empty, buttons A / B drive instead.
    R (top right)        HOLD = autonomy: follows /drive from reactive_node.
                         Let go = back to RC immediately.

The Joy-Cons are paired on the Pi (bluetoothctl). The kernel's hid-nintendo
driver turns each one into /dev/input/event*; this node finds them by name,
and re-scans every few seconds so reconnecting a Joy-Con just works.

Button / axis names are parameters because they depend on the kernel version
and how the Joy-Con is held: run `ros2 run pi_drive joycon_probe`, press the
buttons, and put the names it prints in pi_drive.yaml.

Publishes /drive_cmd [steering, throttle] at publish_hz, always (even zeros),
and /drive_mode (String) for the dashboard.
"""
import math
import os
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from ackermann_msgs.msg import AckermannDriveStamped

try:
    import evdev
    from evdev import ecodes
except ImportError:  # shows a clear error from the node instead of crashing on import
    evdev = None
    ecodes = None


# ///////////////// CONFIGS \\\\\\\\\\\\\\\\\\\\
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
    rc_forward: float = 0.30           # max forward throttle (stick fully forward, or while A is held)
    rc_reverse: float = 0.25           # throttle (magnitude) while B is held
    throttle_deadzone: float = 0.12    # stick deadzone for the drive axis
    throttle_sign: float = 1.0         # raw drive axis -> + = forward. Flip if up drives backwards
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
    throttle_stick: float = 0.0  # raw drive axis, -1..1
    use_axis_throttle: bool = False  # drive from throttle_stick instead of the A / B buttons
    a: bool = False
    b: bool = False
    r: bool = False


# Turns "what the Joy-Cons are doing" + "what the autonomy node last said" into one command: (steering, throttle, mode).
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
        if inp.use_axis_throttle:
            if inp.have_steer:
                t = c.throttle_sign * apply_deadzone(inp.throttle_stick, c.throttle_deadzone)
                throttle = t * c.rc_forward if t > 0 else t * c.rc_reverse
                throttle = clamp(throttle, -c.rc_reverse, c.rc_forward) + 0.0
        elif inp.have_buttons:
            if inp.a and not inp.b:
                throttle = c.rc_forward
            elif inp.b and not inp.a:
                throttle = -c.rc_reverse
        return steering, throttle, 'RC'


class JoyconTeleop(Node):
    def __init__(self):
        super().__init__('joycon_teleop')

        cfg = MixerConfig()
        for k, v in cfg.__dict__.items():
            self.declare_parameter(k, float(v))
        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('poll_hz', 100.0)
        self.declare_parameter('rescan_s', 2.0)
        self.declare_parameter('left_name_contains', ['left joy-con', 'joy-con (l)'])
        self.declare_parameter('right_name_contains', ['right joy-con', 'joy-con (r)'])
        self.declare_parameter('steer_side', 'left')           # which Joy-Con's stick steers
        self.declare_parameter('steer_axis', 'ABS_X')
        self.declare_parameter('throttle_axis', '')            # '' = drive with the A / B buttons
        self.declare_parameter('button_side', 'right')         # which Joy-Con has A / B / R
        self.declare_parameter('a_button', 'BTN_EAST')
        self.declare_parameter('b_button', 'BTN_SOUTH')
        self.declare_parameter('autonomy_button', 'BTN_TR')

        self.mixer = Mixer(MixerConfig(**{k: float(self.get_parameter(k).value) for k in cfg.__dict__}))
        self.left_names = [s.lower() for s in self.get_parameter('left_name_contains').value]
        self.right_names = [s.lower() for s in self.get_parameter('right_name_contains').value]
        self.steer_side = self.get_parameter('steer_side').value
        self.button_side = self.get_parameter('button_side').value

        self.devices = {}      # 'left' / 'right' -> InputDevice
        self.axis_norm = {}    # side -> (mid, half) for the steering axis
        self._ignored = set()  # paths that are not Joy-Cons
        self.inputs = Inputs()
        self.auto_steer = 0.0
        self.auto_speed = 0.0
        self.auto_time = None
        self.mode = None

        self.throttle_code = None
        if evdev is None:
            self.get_logger().error('python3-evdev is not installed; no Joy-Con input')
        else:
            self.steer_code = ecodes.ecodes.get(self.get_parameter('steer_axis').value)
            thr_axis = self.get_parameter('throttle_axis').value
            self.throttle_code = ecodes.ecodes.get(thr_axis) if thr_axis else None
            if thr_axis and self.throttle_code is None:
                self.get_logger().error(f'unknown evdev name for throttle_axis: {thr_axis}')
            self.a_code = ecodes.ecodes.get(self.get_parameter('a_button').value)
            self.b_code = ecodes.ecodes.get(self.get_parameter('b_button').value)
            self.r_code = ecodes.ecodes.get(self.get_parameter('autonomy_button').value)
            for name, code in (('steer_axis', self.steer_code), ('a_button', self.a_code),
                               ('b_button', self.b_code), ('autonomy_button', self.r_code)):
                if code is None:
                    self.get_logger().error(f'unknown evdev name for {name}: {self.get_parameter(name).value}')

        self.cmd_pub = self.create_publisher(Float32MultiArray, '/drive_cmd', 10)
        self.mode_pub = self.create_publisher(String, '/drive_mode', 10)
        self.create_subscription(AckermannDriveStamped, '/drive', self._on_drive, 10)

        self.create_timer(1.0 / float(self.get_parameter('poll_hz').value), self._poll)
        self.create_timer(float(self.get_parameter('rescan_s').value), self._scan)
        self.create_timer(1.0 / float(self.get_parameter('publish_hz').value), self._publish)
        self._scan()
        self.get_logger().info('joycon_teleop up (waiting for Joy-Cons)')

    # ---- autonomy output from reactive_node ----
    def _on_drive(self, msg):
        self.auto_steer = msg.drive.steering_angle
        self.auto_speed = msg.drive.speed
        self.auto_time = self.get_clock().now()

    # ---- device discovery ----
    def _classify(self, name):
        n = name.lower()
        if 'imu' in n:
            return None
        if any(s in n for s in self.left_names):
            return 'left'
        if any(s in n for s in self.right_names):
            return 'right'
        return None

    def _scan(self):
        if evdev is None:
            return
        paths = evdev.list_devices()
        self._ignored &= set(paths)
        have = {dev.path for dev in self.devices.values()}
        for path in paths:
            if path in have or path in self._ignored:
                continue
            try:
                dev = evdev.InputDevice(path)
            except PermissionError:
                self.get_logger().warn(
                    f'no permission to read {path}: add the host "input" group id '
                    'to group_add in docker-compose.pi.yml (INPUT_GID)',
                    throttle_duration_sec=30.0)
                continue
            except OSError:
                continue
            side = self._classify(dev.name)
            if side is None or side in self.devices:
                self._ignored.add(path)
                dev.close()
                continue
            os.set_blocking(dev.fd, False)
            self.devices[side] = dev
            if side == self.steer_side:
                norms = {}
                for code in (self.steer_code, self.throttle_code):
                    if code is None:
                        continue
                    info = dev.absinfo(code)
                    if info is None:
                        self.get_logger().error(f'{dev.name} has no axis with code {code}')
                    else:
                        norms[code] = ((info.max + info.min) / 2.0, max((info.max - info.min) / 2.0, 1.0))
                self.axis_norm[side] = norms
            self.get_logger().info(f'connected {side}: {dev.name} ({path})')

    def _drop(self, side):
        dev = self.devices.pop(side, None)
        if dev is not None:
            self.get_logger().warn(f'{side} Joy-Con disconnected')
            try:
                dev.close()
            except OSError:
                pass
        self.axis_norm.pop(side, None)
        if side == self.steer_side:
            self.inputs.stick = 0.0
            self.inputs.throttle_stick = 0.0
        if side == self.button_side:
            self.inputs.a = self.inputs.b = self.inputs.r = False

    # ---- reading ----
    def _poll(self):
        for side, dev in list(self.devices.items()):
            try:
                for ev in dev.read():
                    self._handle(side, ev)
            except BlockingIOError:
                pass
            except OSError:
                self._drop(side)

    def _handle(self, side, ev):
        if ev.type == ecodes.EV_ABS and side == self.steer_side and ev.code in (self.steer_code, self.throttle_code):
            mid, half = self.axis_norm.get(side, {}).get(ev.code, (0.0, 32767.0))
            value = clamp((ev.value - mid) / half, -1.0, 1.0)
            if ev.code == self.steer_code:
                self.inputs.stick = value
            else:
                self.inputs.throttle_stick = value
        elif ev.type == ecodes.EV_KEY and side == self.button_side:
            down = ev.value != 0
            if ev.code == self.a_code:
                self.inputs.a = down
            elif ev.code == self.b_code:
                self.inputs.b = down
            elif ev.code == self.r_code:
                self.inputs.r = down

    # ---- output ----
    def _publish(self):
        self.inputs.have_steer = self.steer_side in self.devices
        self.inputs.have_buttons = self.button_side in self.devices
        self.inputs.use_axis_throttle = self.throttle_code is not None
        age = (self.get_clock().now() - self.auto_time).nanoseconds / 1e9 if self.auto_time else float('inf')
        steering, throttle, mode = self.mixer.mix(self.inputs, self.auto_steer, self.auto_speed, age)

        if mode != self.mode:
            self.get_logger().info(f'mode: {mode}')
            self.mode = mode
        self.cmd_pub.publish(Float32MultiArray(data=[float(steering), float(throttle)]))
        self.mode_pub.publish(String(data=mode))


def main(args=None):
    rclpy.init(args=args)
    node = JoyconTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

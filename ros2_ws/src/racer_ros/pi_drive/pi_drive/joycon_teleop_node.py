#!/usr/bin/env python3
"""
joycon_teleop_node: Joy-Cons (read straight from Linux evdev) -> /drive_cmd

    stick, one axis      steering (steer_axis)
    stick, other axis    drive: up = forward, down = reverse, proportional (throttle_axis).
                         With throttle_axis empty, buttons A / B drive instead.
    autonomy button      AUTONOMY only while it is held down: the car follows /drive from reactive_node.
                         Let go and it is back in RC at once. Losing the controller also drops back to RC.

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
from std_msgs.msg import Empty, Float32MultiArray, String
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import SetParametersResult

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
    # Speed-sensitive steering: the wheels follow the stick, but never faster than
    #   rate = steer_rate_min + steer_rate_gain * v * (1 + v)      (v = how fast the stick moves, full-lock-units / s)
    # so a slow push builds up slowly and a hard flick gets to full lock almost instantly.
    # steer_rate_gain 0 = off (wheels jump straight to the stick).
    steer_rate_min: float = 0.25       # slowest the wheels may move (full lock per s); also how fast they settle when the stick stops
    steer_rate_gain: float = 0.8       # bigger = follows the stick more eagerly
    steer_vel_hold_s: float = 0.25     # a fast flick keeps the fast rate this long, so the wheels can finish the move
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
        self._target = self._out = 0.0     # stick steering wanted / steering actually sent
        self._vel = 0.0                    # recent stick speed (peak, decaying), full-lock-units / s

    def _shape_steer(self, target, dt):
        """Slew-limit the steering by how fast the stick is being moved."""
        c = self.cfg
        if dt is None or dt <= 0.0 or c.steer_rate_gain <= 0.0:
            self._target = self._out = target
            self._vel = 0.0
            return target
        v_now = abs(target - self._target) / dt
        decay = math.exp(-dt / max(c.steer_vel_hold_s, 1e-3))
        self._vel = max(v_now, self._vel * decay)
        self._target = target
        rate = c.steer_rate_min + c.steer_rate_gain * self._vel * (1.0 + self._vel)
        step = rate * dt
        self._out = target if abs(target - self._out) <= step else self._out + math.copysign(step, target - self._out)
        return self._out + 0.0

    def mix(self, inp: Inputs, auto_steer_rad: float = 0.0,
            auto_speed: float = 0.0, auto_age_s: float = float('inf'), dt=None):
        c = self.cfg

        if not inp.have_steer and not inp.have_buttons:
            self._shape_steer(0.0, None)
            return 0.0, 0.0, 'NO_CONTROLLER'

        if inp.have_buttons and inp.r:
            if auto_age_s > c.auto_timeout_s:
                self._shape_steer(0.0, None)
                return 0.0, 0.0, 'AUTO_WAITING'
            steering = clamp(auto_steer_rad / c.auto_max_steer_rad, -1.0, 1.0)
            self._shape_steer(steering, None)      # back to RC later starts from where autonomy left the wheels
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
        steering = self._shape_steer(steering, dt)

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
        self.declare_parameter('mute_button', 'BTN_DPAD_UP')    # toggles the speaker mute; '' = off

        self.mixer = Mixer(MixerConfig(**{k: float(self.get_parameter(k).value) for k in cfg.__dict__}))
        self.left_names = [s.lower() for s in self.get_parameter('left_name_contains').value]
        self.right_names = [s.lower() for s in self.get_parameter('right_name_contains').value]
        self.steer_side = self.get_parameter('steer_side').value
        self.button_side = self.get_parameter('button_side').value

        self.add_on_set_parameters_callback(self._on_params)   # lets the dashboard Tuning tab change these live
        self.devices = {}      # 'left' / 'right' -> InputDevice
        self.axis_norm = {}    # side -> (mid, half) for the steering axis
        self._ignored = set()  # paths that are not Joy-Cons
        self.inputs = Inputs()
        self.auto_steer = 0.0
        self.auto_speed = 0.0
        self.auto_time = None
        self.mode = None
        self.last_pub_t = None
        self.btn_auto_down = False  # the autonomy button is held down = AUTONOMY, released = RC

        self.throttle_code = None
        self.mute_code = None
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
            self.mute_code = ecodes.ecodes.get(self.get_parameter('mute_button').value)
            for name, code in (('steer_axis', self.steer_code), ('a_button', self.a_code),
                               ('b_button', self.b_code), ('autonomy_button', self.r_code)):
                if code is None:
                    self.get_logger().error(f'unknown evdev name for {name}: {self.get_parameter(name).value}')

        self.cmd_pub = self.create_publisher(Float32MultiArray, '/drive_cmd', 10)
        self.mode_pub = self.create_publisher(String, '/drive_mode', 10)
        self.mute_pub = self.create_publisher(Empty, '/mute', 10)
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
            self.btn_auto_down = False
            self.get_logger().warn('controller lost: back to RC')

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
            elif ev.code == self.mute_code and ev.value == 1:     # on the press only, not the hold / release
                self.mute_pub.publish(Empty())
            elif ev.code == self.r_code:
                self.btn_auto_down = down

    # parameters the dashboard may change while the car runs, with their allowed range
    LIVE_RANGES = {'rc_forward': (0.0, 1.0), 'rc_reverse': (0.0, 1.0), 'auto_max_throttle': (0.0, 1.0),
                   'steer_deadzone': (0.0, 0.9), 'throttle_deadzone': (0.0, 0.9),
                   'steer_rate_min': (0.02, 20.0), 'steer_rate_gain': (0.0, 20.0), 'steer_vel_hold_s': (0.01, 2.0)}

    def _on_params(self, params):
        for p in params:
            if p.name not in self.LIVE_RANGES:
                continue
            lo, hi = self.LIVE_RANGES[p.name]
            try:
                v = float(p.value)
            except (TypeError, ValueError):
                return SetParametersResult(successful=False, reason=f'{p.name} must be a number')
            if not lo <= v <= hi:
                return SetParametersResult(successful=False, reason=f'{p.name} must be between {lo} and {hi}')
        for p in params:
            if p.name in self.LIVE_RANGES:
                setattr(self.mixer.cfg, p.name, float(p.value))
                self.get_logger().info(f'{p.name} changed to {float(p.value):g}')
        return SetParametersResult(successful=True)

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ---- output ----
    def _publish(self):
        self.inputs.have_steer = self.steer_side in self.devices
        self.inputs.have_buttons = self.button_side in self.devices
        self.inputs.use_axis_throttle = self.throttle_code is not None
        if not (self.inputs.have_steer or self.inputs.have_buttons):
            self.btn_auto_down = False                # no controller: never autonomy
        self.inputs.r = self.btn_auto_down            # the mixer's autonomy flag = the button is held
        age = (self.get_clock().now() - self.auto_time).nanoseconds / 1e9 if self.auto_time else float('inf')
        now = self._now_s()
        dt = min(now - self.last_pub_t, 0.2) if self.last_pub_t is not None else None
        self.last_pub_t = now
        steering, throttle, mode = self.mixer.mix(self.inputs, self.auto_steer, self.auto_speed, age, dt)

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

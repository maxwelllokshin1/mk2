#!/usr/bin/env python3
"""
joycon_teleop_node: Joy-Cons (read straight from Linux evdev) -> /drive_cmd

    left Joy-Con stick   steering
    A (right Joy-Con)    drive forward while held
    B (right Joy-Con)    reverse while held
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
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from ackermann_msgs.msg import AckermannDriveStamped

from pi_drive.mixer import Inputs, Mixer, MixerConfig, clamp

try:
    import evdev
    from evdev import ecodes
except ImportError:  # shows a clear error from the node instead of crashing on import
    evdev = None
    ecodes = None


class JoyconTeleop(Node):
    def __init__(self):
        super().__init__('joycon_teleop')

        d = self.declare_parameter
        cfg = MixerConfig()
        for k, v in cfg.__dict__.items():
            d(k, float(v))
        d('publish_hz', 30.0)
        d('poll_hz', 100.0)
        d('rescan_s', 2.0)
        d('left_name_contains', ['left joy-con', 'joy-con (l)'])
        d('right_name_contains', ['right joy-con', 'joy-con (r)'])
        d('steer_side', 'left')           # which Joy-Con's stick steers
        d('steer_axis', 'ABS_X')
        d('button_side', 'right')         # which Joy-Con has A / B / R
        d('a_button', 'BTN_EAST')
        d('b_button', 'BTN_SOUTH')
        d('autonomy_button', 'BTN_TR')

        p = lambda n: self.get_parameter(n).value
        self.mixer = Mixer(MixerConfig(**{k: float(p(k)) for k in cfg.__dict__}))
        self.left_names = [s.lower() for s in p('left_name_contains')]
        self.right_names = [s.lower() for s in p('right_name_contains')]
        self.steer_side = p('steer_side')
        self.button_side = p('button_side')

        self.devices = {}      # 'left' / 'right' -> InputDevice
        self.axis_norm = {}    # side -> (mid, half) for the steering axis
        self._ignored = set()  # paths that are not Joy-Cons
        self.inputs = Inputs()
        self.auto_steer = 0.0
        self.auto_speed = 0.0
        self.auto_time = None
        self.mode = None

        if evdev is None:
            self.get_logger().error('python3-evdev is not installed; no Joy-Con input')
        else:
            self.steer_code = ecodes.ecodes.get(p('steer_axis'))
            self.a_code = ecodes.ecodes.get(p('a_button'))
            self.b_code = ecodes.ecodes.get(p('b_button'))
            self.r_code = ecodes.ecodes.get(p('autonomy_button'))
            for name, code in (('steer_axis', self.steer_code), ('a_button', self.a_code),
                               ('b_button', self.b_code), ('autonomy_button', self.r_code)):
                if code is None:
                    self.get_logger().error(f'unknown evdev name for {name}: {p(name)}')

        self.cmd_pub = self.create_publisher(Float32MultiArray, '/drive_cmd', 10)
        self.mode_pub = self.create_publisher(String, '/drive_mode', 10)
        self.create_subscription(AckermannDriveStamped, '/drive', self._on_drive, 10)

        self.create_timer(1.0 / float(p('poll_hz')), self._poll)
        self.create_timer(float(p('rescan_s')), self._scan)
        self.create_timer(1.0 / float(p('publish_hz')), self._publish)
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
            if side == self.steer_side and self.steer_code is not None:
                info = dev.absinfo(self.steer_code)
                if info is None:
                    self.get_logger().error(f'{dev.name} has no axis {self.get_parameter("steer_axis").value}')
                else:
                    self.axis_norm[side] = ((info.max + info.min) / 2.0, max((info.max - info.min) / 2.0, 1.0))
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
        if ev.type == ecodes.EV_ABS and side == self.steer_side and ev.code == self.steer_code:
            mid, half = self.axis_norm.get(side, (0.0, 32767.0))
            self.inputs.stick = clamp((ev.value - mid) / half, -1.0, 1.0)
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

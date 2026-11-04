#!/usr/bin/env python3
"""
oled_display_node: draws the current expression on the 128x64 I2C OLED(s).

The expression comes from expression_node on /expression (latched). How each face is built (eyeball under
EVERY face, then iris / eyelid / loops / blink) is described in expressions.py and eye_plan().

    /expression          String   which face to show
    /expression_closing  String   the face is about to end: eyelid faces (angry, bored) play the lid backwards
    /person              PointStamped  iris follows the person (faces that track), within what the lid leaves open
    /oled/layer_xy       geometry_msgs/Point   move the iris yourself (display pixels)

Two screens: screen 1 shows the pictures as drawn, screen 2 shows the eyelid mirrored (mirror_screen: 2).
A screen that is not there is retried every 5 s; with no screen at all the node keeps running and can still
write a preview PNG (preview_path).
"""
import os
import random
import time
from types import SimpleNamespace

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from pi_drive.expressions import EXPRESSIONS, eye_plan, look_mode
from pi_drive.oled_device import MultiDisplay, NullDisplay
from pi_drive.oled_layers import Compositor, ease, load_layer, nearest, open_centers, person_to_center


class OledDisplay(Node):
    def __init__(self):
        super().__init__('oled_display')

        d = self.declare_parameter
        d('expressions_dir', '')                             # '' = the package config/expressions folder
        d('start_expression', 'normal')
        d('blink_every_s', 10.0)                             # blinking expressions blink this often (0 = never)
        d('iris_size', [44, 44])                             # iris.png is resized to this
        d('move_limits', [30, 98, 16, 48])                   # x_min, x_max, y_min, y_max of the iris center
        d('lid_move_limits', [20, 108, 10, 60])             # eyelid faces (angry, bored): the iris may go lower / wider, but only where the lid leaves it visible
        d('lid_overlap_px', 20)                              # ... this many iris pixels may touch the lid
        d('wander_min_s', 1.0)                               # bored: look somewhere else every 1-3 s
        d('wander_max_s', 3.0)
        d('variant_min_s', 1.5)                              # excited: flips between its two looks every 1.5-4 s
        d('variant_max_s', 4.0)
        d('mirror_screen', 2)                                # which screen gets the mirrored eyelid (1 or 2)
        d('render_hz', 15.0)
        d('threshold', 160)
        d('dither', False)
        d('black_transparent', False)
        d('track_person', True)
        d('track_invert_x', True)
        d('track_invert_y', False)
        d('track_smooth', 0.35)                              # 0..1 per frame, 1 = jump
        d('lost_timeout_s', 2.0)
        d('i2c_bus', 1)
        d('i2c_address', 0x3C)
        d('second_i2c_address', 0x3D)                        # second screen. 0 = single screen
        d('second_i2c_bus', -1)                              # -1 = same bus as i2c_bus
        d('driver', 'ssd1306')                               # or sh1106
        d('rotate', 0)
        d('preview_path', '')                                # e.g. /tmp/oled.png, written once a second

        p = lambda n: self.get_parameter(n).value
        self.root = p('expressions_dir') or os.path.join(get_package_share_directory('pi_drive'), 'config', 'expressions')
        self.blink_every = float(p('blink_every_s'))
        self.iris_size = tuple(int(v) for v in p('iris_size'))
        self.threshold, self.dither, self.black_transparent = int(p('threshold')), bool(p('dither')), bool(p('black_transparent'))
        self.mirror_second = int(p('mirror_screen')) != 1
        self.limits = tuple(float(v) for v in p('move_limits'))
        self.lid_limits = tuple(float(v) for v in p('lid_move_limits'))
        self.lid_overlap = int(p('lid_overlap_px'))

        # every expression is loaded once at start: name -> face (see _build)
        self.faces = {name: self._build(name) for name in EXPRESSIONS}

        self.pos = self.target = (64.0, 32.0)
        self.smooth = float(p('track_smooth'))
        self.track = bool(p('track_person'))
        self.inv_x, self.inv_y = bool(p('track_invert_x')), bool(p('track_invert_y'))
        self.lost_timeout = float(p('lost_timeout_s'))
        self.last_person = None
        self.preview = p('preview_path')
        self.last_preview = 0.0
        self.last_open_try = 0.0
        self.wander_range = (float(p('wander_min_s')), float(p('wander_max_s')))
        self.variant_range = (float(p('variant_min_s')), float(p('variant_max_s')))
        self.next_wander = self.next_variant = 0.0
        self.expression = None
        self._set_expression(str(p('start_expression')))

        self.display = NullDisplay()
        self._open_display()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Point, '/oled/layer_xy', self._on_xy, 10)
        self.create_subscription(PointStamped, '/person', self._on_person, 10)
        self.create_subscription(String, '/expression', lambda m: self._set_expression(m.data), latched)
        self.create_subscription(String, '/expression_closing', self._on_closing, 10)
        self.create_timer(1.0 / float(p('render_hz')), self._render)
        self.get_logger().info(f'oled_display up: {len(self.faces)} expressions, display live={self.display.live}')

    # ---------- building the faces ----------
    def _load(self, path, size, mirror=False):
        try:
            return load_layer(path, size, (64, 32), self.black_transparent, mirror)
        except Exception as exc:     # a bad image should not kill the node
            self.get_logger().error(f'{path} not loaded: {exc}')
            return None

    def _build(self, name):
        """
        face = looks (one per variant: [(screen-1 Compositor, screen-2 Compositor, iris layer index or -1)]),
               allowed iris centres (None = anywhere inside move_limits), rest = where the iris sits when idle.
        Layers bottom -> top: eyeball, iris, looping gifs, the variant, eyelid, blink.
        """
        spec, plan = EXPRESSIONS[name], eye_plan(self.root, name)
        if plan['base'] is None:
            self.get_logger().warn(f'{name}: normal/eyes/base.png is missing, the screen will stay black')
        full = (128, 64)
        base = self._load(plan['base'], full) if plan['base'] else None
        iris = self._load(plan['iris'], self.iris_size) if plan['iris'] else None
        loops = [l for l in (self._load(f, full) for f in plan['loop']) if l]
        lid = self._load(plan['lid'], full) if plan['lid'] else None
        lid_mirror = self._load(plan['lid'], full, mirror=True) if plan['lid'] else None
        for layer in (lid, lid_mirror):
            if layer is not None:
                layer.hold = True                                          # play once, hold, reverse when closing
        blink = self._load(plan['blink'], full) if plan['blink'] and self.blink_every > 0 else None
        if blink is not None:
            blink.gap = max(self.blink_every - blink.duration, 0.0)        # one blink every blink_every_s ...
            blink.t_offset = blink.duration                                # ... the first one after a full gap

        def stack(variant, the_lid):
            layers = [l for l in (base, iris, *loops, variant, the_lid, blink) if l is not None]
            return Compositor(layers, full, [layers.index(iris)] if iris is not None else [], 0.0,
                              self.threshold, self.dither), (layers.index(iris) if iris is not None else -1)

        looks = []
        for files in plan['variants'] or [[]]:
            variant = self._load(files[0], full) if files else None
            comp1, iris_idx = stack(variant, lid)
            comp2 = stack(variant, lid_mirror)[0] if lid is not None else comp1
            looks.append((comp2, comp1, iris_idx) if (lid is not None and not self.mirror_second) else (comp1, comp2, iris_idx))

        allowed, rest = None, (64.0, 32.0)
        if lid is not None and iris is not None:
            # screen 1 and 2 have mirrored lids, so the iris stays where BOTH leave it visible: same spot on both eyes
            allowed = open_centers(iris, [lid, lid_mirror], self.lid_limits, tolerance=self.lid_overlap) or None
            if allowed:
                rest = nearest(allowed, (64.0, 56.0))        # bottom middle of what the lid leaves open
            else:
                self.get_logger().warn(f'{name}: the eyelid leaves no room for the iris, it will stay hidden')
        return SimpleNamespace(looks=looks, allowed=allowed, rest=rest)

    def _set_expression(self, name):
        if name not in self.faces:
            self.get_logger().warn(f'unknown expression: {name}')
            return
        if name == self.expression:
            return
        now = time.monotonic()
        self.expression, self.face = name, self.faces[name]
        self.look = look_mode(name)
        self.look_i = random.randrange(len(self.face.looks))
        self.next_variant = now + random.uniform(*self.variant_range)
        self.next_wander = now
        self.target = self.face.rest
        self.t0, self.closing_t = now, None
        self.last_bytes = None
        self.get_logger().info(f'expression: {name}')

    def _on_closing(self, msg):
        if msg.data == self.expression:
            self.closing_t = time.monotonic() - self.t0

    def _open_display(self):
        self.last_open_try = time.monotonic()
        p = lambda n: self.get_parameter(n).value
        try:
            bus, rot = int(p('i2c_bus')), int(p('rotate'))
            specs = [(bus, int(p('i2c_address')), p('driver'), rot)]
            if int(p('second_i2c_address')):
                bus2 = int(p('second_i2c_bus'))
                specs.append((bus if bus2 < 0 else bus2, int(p('second_i2c_address')), p('driver'), rot))
            self.display = MultiDisplay(specs)
            self.last_bytes = None
            self.get_logger().info(f'OLED connected: {len(self.display.devices)}/{len(specs)} screens')
        except Exception as exc:
            self.display = NullDisplay()
            self.get_logger().warn(f'OLED not available ({exc}); retrying every 5 s',
                                   throttle_duration_sec=30.0)

    def _retry_missing(self):
        """A screen that was not there at start (loose wire, powered late) is picked up when it shows up."""
        self.last_open_try = time.monotonic()
        if self.display.retry():
            self.last_bytes = None
            self.get_logger().info(f'OLED screen came up: {len(self.display.devices)}/{len(self.display.specs)} screens')
        for err in self.display.errors.values():
            self.get_logger().warn(f'OLED screen missing, {err}', throttle_duration_sec=30.0)

    def _snap(self, xy):
        """Inside move_limits, and (eyelid faces) only where the lid leaves the iris visible."""
        if self.face.allowed:
            return nearest(self.face.allowed, xy)
        x_min, x_max, y_min, y_max = self.limits
        return min(max(xy[0], x_min), x_max), min(max(xy[1], y_min), y_max)

    def _on_xy(self, msg):
        self.target = self._snap((msg.x, msg.y))

    def _on_person(self, msg):
        self.last_person = time.monotonic()
        if self.track and self.look == 'track':
            self.target = self._snap(person_to_center(msg.point.x, msg.point.y, self.limits, self.inv_x, self.inv_y))

    def _render(self):
        now = time.monotonic()
        if self.look == 'wander':
            if now >= self.next_wander:
                x_min, x_max, y_min, y_max = self.limits
                self.target = random.choice(self.face.allowed) if self.face.allowed else (
                    random.uniform(x_min, x_max), random.uniform(y_min, y_max))
                self.next_wander = now + random.uniform(*self.wander_range)
        elif self.look == 'none':
            self.target = self.face.rest
        elif self.track and self.last_person is not None and now - self.last_person > self.lost_timeout:
            self.target = self.face.rest
            self.last_person = None
        self.pos = self._snap(ease(self.pos, self.target, self.smooth))

        if len(self.face.looks) > 1 and now >= self.next_variant:       # excited: flip to the other look
            self.look_i = (self.look_i + 1) % len(self.face.looks)
            self.next_variant = now + random.uniform(*self.variant_range)
        comp1, comp2, iris = self.face.looks[self.look_i]
        t = now - self.t0
        positions = {iris: self.pos} if iris >= 0 else {}
        img1 = comp1.render(t, positions, self.closing_t)
        img2 = img1 if comp2 is comp1 else comp2.render(t, positions, self.closing_t)

        data = img1.tobytes() + img2.tobytes()
        if data != self.last_bytes:                       # skip identical frames: no I2C traffic
            try:
                self.display.show([img1, img2])
                self.last_bytes = data
            except Exception as exc:
                self.get_logger().error(f'OLED write failed: {exc}', throttle_duration_sec=10.0)
                self.display = NullDisplay()
        if now - self.last_open_try > 5.0:
            if not self.display.live:
                self._open_display()
            elif self.display.missing:
                self._retry_missing()

        if self.preview and now - self.last_preview > 1.0:
            self.last_preview = now
            try:
                img1.convert('L').resize((512, 256)).save(self.preview)
            except Exception as exc:
                self.get_logger().warn(f'preview not saved: {exc}', throttle_duration_sec=30.0)

    def shutdown(self):
        self.display.close()


def main(args=None):
    rclpy.init(args=args)
    node = OledDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
oled_display_node: draws layered images (PNG / animated GIF) on the 128x64 I2C OLED.

    layers (bottom -> top) from the `layers` parameter, e.g.
        eyeball.png   white of the eye
        eye.png       iris / pupil  <- the MOVABLE layer (movable_layer)
        blink.gif     eyelid, plays once every gif_gap_s seconds

Moving the movable layer:
    /oled/layer_xy   geometry_msgs/Point   x, y = new CENTER in display pixels (0..127, 0..63)
    /person          geometry_msgs/PointStamped from person_detector; when track_person is true
                     the layer follows the person (eyes look at them), and returns to its rest
                     position `lost_timeout_s` after the person disappears.
Both are clamped to move_limits and eased so the motion glides.

If the screen is not there (or I2C is off), the node keeps running, retries every 5 s, and can
still write a preview PNG (preview_path) so you can see what it would show.
"""
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from ament_index_python.packages import get_package_share_directory

from pi_drive.oled_device import LumaDisplay, NullDisplay
from pi_drive.oled_layers import Compositor, ease, load_layer, person_to_center


class OledDisplay(Node):
    def __init__(self):
        super().__init__('oled_display')

        d = self.declare_parameter
        d('assets_dir', '')                                  # '' = the package config dir
        d('layers', ['eyeball.png', 'eye.png', 'blink.gif'])
        d('layer_sizes', [128, 64, 44, 44, 128, 64])         # w,h per layer, in pixels
        d('layer_positions', [64, 32, 64, 32, 64, 32])       # center x,y per layer (rest position)
        d('movable_layer', 1)                                # index in `layers`, -1 = none
        d('move_limits', [30, 98, 16, 48])                   # x_min, x_max, y_min, y_max of its center
        d('gif_gap_s', 4.0)                                  # GIFs play once every this many s (0 = loop)
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
        d('driver', 'ssd1306')                               # or sh1106
        d('rotate', 0)
        d('preview_path', '')                                # e.g. /tmp/oled.png, written once a second

        p = lambda n: self.get_parameter(n).value
        assets = p('assets_dir') or os.path.join(get_package_share_directory('pi_drive'), 'config')
        names = list(p('layers'))
        sizes, positions = list(p('layer_sizes')), list(p('layer_positions'))
        if len(sizes) < 2 * len(names) or len(positions) < 2 * len(names):
            raise ValueError('layer_sizes and layer_positions need 2 numbers (x,y / w,h) per layer')

        layers, self.movable = [], -1
        for i, name in enumerate(names):
            path = name if os.path.isabs(name) else os.path.join(assets, name)
            try:
                layers.append(load_layer(path, (sizes[2 * i], sizes[2 * i + 1]),
                                         (positions[2 * i], positions[2 * i + 1]), p('black_transparent')))
            except Exception as exc:    # a missing image should not kill the node
                self.get_logger().error(f'layer {i} ({path}) not loaded: {exc}')
                continue
            if i == int(p('movable_layer')):
                self.movable = len(layers) - 1      # index among the layers that did load
        self.comp = Compositor(layers, (128, 64), [self.movable] if self.movable >= 0 else [],
                               float(p('gif_gap_s')), int(p('threshold')), bool(p('dither')))

        self.limits = tuple(float(v) for v in p('move_limits'))
        self.rest = layers[self.movable].center if self.movable >= 0 else (64.0, 32.0)
        self.pos = self.target = tuple(float(v) for v in self.rest)
        self.smooth = float(p('track_smooth'))
        self.track = bool(p('track_person'))
        self.inv_x, self.inv_y = bool(p('track_invert_x')), bool(p('track_invert_y'))
        self.lost_timeout = float(p('lost_timeout_s'))
        self.last_person = None
        self.preview = p('preview_path')
        self.t0 = time.monotonic()
        self.last_bytes = None
        self.last_preview = 0.0
        self.last_open_try = 0.0

        self.display = NullDisplay()
        self._open_display()

        self.create_subscription(Point, '/oled/layer_xy', self._on_xy, 10)
        self.create_subscription(PointStamped, '/person', self._on_person, 10)
        self.create_timer(1.0 / float(p('render_hz')), self._render)
        self.get_logger().info(
            f'oled_display up: {len(layers)} layers, movable={self.movable}, display live={self.display.live}')

    def _open_display(self):
        self.last_open_try = time.monotonic()
        p = lambda n: self.get_parameter(n).value
        try:
            self.display = LumaDisplay(int(p('i2c_bus')), int(p('i2c_address')), p('driver'), int(p('rotate')))
            self.last_bytes = None
            self.get_logger().info('OLED connected')
        except Exception as exc:
            self.display = NullDisplay()
            self.get_logger().warn(f'OLED not available ({exc}); retrying every 5 s',
                                   throttle_duration_sec=30.0)

    def _clamp(self, xy):
        x_min, x_max, y_min, y_max = self.limits
        return min(max(xy[0], x_min), x_max), min(max(xy[1], y_min), y_max)

    def _on_xy(self, msg):
        self.target = self._clamp((msg.x, msg.y))

    def _on_person(self, msg):
        self.last_person = time.monotonic()
        if self.track:
            self.target = person_to_center(msg.point.x, msg.point.y, self.limits, self.inv_x, self.inv_y)

    def _render(self):
        now = time.monotonic()
        if self.track and self.last_person is not None and now - self.last_person > self.lost_timeout:
            self.target = self.rest
            self.last_person = None
        self.pos = ease(self.pos, self.target, self.smooth)

        positions = {self.movable: self.pos} if self.movable >= 0 else {}
        img = self.comp.render(now - self.t0, positions)

        data = img.tobytes()
        if data != self.last_bytes:                       # skip identical frames: no I2C traffic
            try:
                self.display.show(img)
                self.last_bytes = data
            except Exception as exc:
                self.get_logger().error(f'OLED write failed: {exc}', throttle_duration_sec=10.0)
                self.display = NullDisplay()
        if not self.display.live and now - self.last_open_try > 5.0:
            self._open_display()

        if self.preview and now - self.last_preview > 1.0:
            self.last_preview = now
            try:
                img.convert('L').resize((512, 256)).save(self.preview)
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

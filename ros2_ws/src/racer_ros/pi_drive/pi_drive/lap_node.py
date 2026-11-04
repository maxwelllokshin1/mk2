#!/usr/bin/env python3
"""
lap_node: counts laps from the LiDAR and publishes /lap_completed (std_msgs/Empty) once per lap,
which makes expression_node go EXCITED. Also publishes /lap_count (Int32).

    1. Park the car on the start line, facing the way it will drive. Parked scans are averaged.
    2. The first time it starts driving (or after it sat still for rearm_idle_s) that picture becomes
       the start-line reference and the count restarts at 0.
    3. Each time the LiDAR sees the start-line picture again after the car really went around, that is a lap.

Inputs: /scan (LaserScan), /drive_cmd (to know when the car is driving), /lap_reset (Empty: forget the
reference, the next start becomes the new start line).
Tuning: ros2 topic echo /lap_debug (distance to the start line in metres; a lap fires below match_m after
it has been above away_m). If laps are missed raise match_m; if it fires early/often raise away_m or min_lap_s.
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, Float32, Float32MultiArray, Int32

from pi_drive.lap_logic import LapDetector, signature


class LapNode(Node):
    def __init__(self):
        super().__init__('lap_detector')
        d = self.declare_parameter
        d('n_bins', 36)                 # directions per scan signature
        d('max_range_m', 6.0)           # ranges are clipped here (far walls / no return all look the same)
        d('match_m', 0.6)               # closer than this to the start-line picture = back at the start
        d('away_m', 1.2)                # must have been farther than this since the last lap
        d('min_lap_s', 8.0)             # never count two laps closer together than this
        d('min_bins', 8)                # directions both scans must share to compare them
        d('process_hz', 10.0)           # scans analysed per second
        d('drive_throttle_min', 0.05)
        d('drive_hold_s', 1.0)          # still "driving" this long after the throttle drops
        d('rearm_idle_s', 20.0)         # parked this long -> the next start sets a new start line + resets the count

        p = lambda n: self.get_parameter(n).value
        self.n_bins, self.max_range = int(p('n_bins')), float(p('max_range_m'))
        self.thr_min, self.hold = float(p('drive_throttle_min')), float(p('drive_hold_s'))
        self.rearm_idle = float(p('rearm_idle_s'))
        self.period = 1.0 / float(p('process_hz'))
        self.det = LapDetector(float(p('match_m')), float(p('away_m')), float(p('min_lap_s')), int(p('min_bins')))

        self.last_drive = None
        self.was_driving = False
        self.parked_since = time.monotonic()
        self.fresh = True                # no start line chosen yet
        self.last_scan_t = 0.0

        self.lap_pub = self.create_publisher(Empty, '/lap_completed', 10)
        self.count_pub = self.create_publisher(Int32, '/lap_count', 10)
        self.debug_pub = self.create_publisher(Float32, '/lap_debug', 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(Float32MultiArray, '/drive_cmd', self._on_cmd, 10)
        self.create_subscription(Empty, '/lap_reset', self._on_reset, 10)
        self.get_logger().info('lap_detector up: park on the start line, then drive')

    def _on_cmd(self, msg):
        if len(msg.data) >= 2 and abs(msg.data[1]) > self.thr_min:
            self.last_drive = time.monotonic()

    def _on_reset(self, _msg):
        self.det.disarm()
        self.fresh = True
        self.get_logger().info('lap reset: the next start becomes the start line')

    def _on_scan(self, msg):
        now = time.monotonic()
        if now - self.last_scan_t < self.period:
            return
        self.last_scan_t = now
        sig = signature(msg.ranges, msg.angle_min, msg.angle_increment, self.n_bins, self.max_range)

        driving = self.last_drive is not None and now - self.last_drive < self.hold
        if not driving:
            if self.was_driving:
                self.parked_since = now
                self.det.standby.clear()          # old pictures are from the track, not the start line
            self.was_driving = False
            self.det.standby_scan(sig)
            return

        if not self.was_driving:                  # just started driving
            self.was_driving = True
            if self.fresh or now - self.parked_since >= self.rearm_idle:
                if self.det.arm(now):
                    self.fresh = False
                    self.count_pub.publish(Int32(data=0))
                    self.get_logger().info('start line saved, counting laps')
        if self.det.update(sig, now):
            self.get_logger().info(f'LAP {self.det.laps}')
            self.lap_pub.publish(Empty())
            self.count_pub.publish(Int32(data=self.det.laps))
        self.debug_pub.publish(Float32(data=float(self.det.dist) if self.det.dist != float('inf') else -1.0))


def main(args=None):
    rclpy.init(args=args)
    node = LapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

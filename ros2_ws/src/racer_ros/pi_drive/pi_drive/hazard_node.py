#!/usr/bin/env python3
"""
hazard_node: watches the LiDAR straight ahead while the car drives forward.

    /surprise    Empty   about to run into something (it is close and getting closer fast) -> SURPRISED face
    /collision   Empty   something is almost touching the lidar                             -> DIZZY face
    /front_range Float32 nearest thing straight ahead in metres (-1 = nothing), for tuning

Inputs: /scan, /drive_cmd (forward throttle > drive_throttle_min counts as driving forward).
Tuning on the track: ros2 topic echo /front_range, drive at a wall, and set
surprise_m / closing_mps / collision_m from what you see (collision_m also depends on how far the lidar sits behind the bumper).
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, Float32, Float32MultiArray

from pi_drive.hazard_logic import HazardDetector, front_range


class HazardNode(Node):
    def __init__(self):
        super().__init__('hazard')
        d = self.declare_parameter
        d('cone_deg', 20.0)               # straight ahead +- this many degrees
        d('surprise_m', 0.9)              # closer than this AND closing in fast = about to hit
        d('closing_mps', 0.5)             # "fast" = the gap shrinks at least this many m/s
        d('collision_m', 0.2)             # closer than this = hit it
        d('surprise_cooldown_s', 8.0)
        d('collision_cooldown_s', 10.0)
        d('while_stationary', True)       # also react when the car is parked and something rushes at it
        d('drive_throttle_min', 0.05)     # forward throttle above this = driving forward
        d('drive_hold_s', 0.5)
        d('process_hz', 20.0)

        p = lambda n: self.get_parameter(n).value
        self.cone = float(p('cone_deg'))
        self.thr_min, self.hold = float(p('drive_throttle_min')), float(p('drive_hold_s'))
        self.period = 1.0 / float(p('process_hz'))
        self.det = HazardDetector(float(p('surprise_m')), float(p('closing_mps')), float(p('collision_m')),
                                  float(p('surprise_cooldown_s')), float(p('collision_cooldown_s')),
                                  while_stationary=bool(p('while_stationary')))
        self.last_forward = None
        self.last_scan_t = 0.0

        self.surprise_pub = self.create_publisher(Empty, '/surprise', 10)
        self.collision_pub = self.create_publisher(Empty, '/collision', 10)
        self.range_pub = self.create_publisher(Float32, '/front_range', 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(Float32MultiArray, '/drive_cmd', self._on_cmd, 10)
        self.get_logger().info(f'hazard up: surprise / collision from the LiDAR (parked too: {self.det.while_stationary})')

    def _on_cmd(self, msg):
        if len(msg.data) >= 2 and msg.data[1] > self.thr_min:
            self.last_forward = time.monotonic()

    def _on_scan(self, msg):
        now = time.monotonic()
        if now - self.last_scan_t < self.period:
            return
        self.last_scan_t = now
        front = front_range(msg.ranges, msg.angle_min, msg.angle_increment, self.cone)
        forward = self.last_forward is not None and now - self.last_forward < self.hold
        event = self.det.update(now, front, forward)
        self.range_pub.publish(Float32(data=front if front != float('inf') else -1.0))
        if event == 'collision':
            self.get_logger().info(f'COLLISION (front {front:.2f} m)')
            self.collision_pub.publish(Empty())
        elif event == 'surprise':
            self.get_logger().info(f'SURPRISE (front {front:.2f} m, closing)')
            self.surprise_pub.publish(Empty())


def main(args=None):
    rclpy.init(args=args)
    node = HazardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

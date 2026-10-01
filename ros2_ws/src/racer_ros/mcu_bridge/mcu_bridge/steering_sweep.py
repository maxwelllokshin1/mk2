#!/usr/bin/env python3
"""
steering_sweep.py — a fake reactive_node for bring-up.

WHY IT EXISTS
    To test steering you'd normally need the lidar, reactive_node and a room to
    drive in. This node publishes /drive by itself, with speed pinned at 0 and
    the steering stepping through  0 -> +A -> 0 -> -A  so you can watch the
    wheels move and check: left is left, the angle is right, the limits hold.
    It exercises the exact same path (/drive -> bridge_node -> backend ->
    hardware) as the real thing.

PARAMETERS
    amplitude_deg  8.0   size of the swing. START SMALL; raise it only after
                         you've confirmed the servo limits in the firmware config.
    hold_s         2.0   seconds to hold each of the four positions.

HOW IT WORKS
    A timer publishes at 20 Hz. It MUST publish continuously, not once per
    change: the firmware watchdog (and bridge_node's cmd_timeout) treat silence
    as "the Pi died" and re-center the wheels within 200-250 ms. A single
    message would only move the wheels for a blink.

    step   = int(seconds_since_start / hold_s) % 4
    target = [0, +amp, 0, -amp][step]        in RADIANS (message units)
    Log the target only when `step` changes, or you'll print 20 lines a second.
    speed is always 0.0.

WHAT THE MESSAGE NEEDS
    AckermannDriveStamped:  header.stamp = now, header.frame_id = 'base_link',
    drive.steering_angle = target (radians, + = left), drive.speed = 0.0
"""
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class SteeringSweep(Node):
    def __init__(self):
        super().__init__('steering_sweep')
        # TODO: declare amplitude_deg and hold_s; create the publisher on '/drive'
        #       (QoS depth 10); store the start time (self.get_clock().now());
        #       remember the last step (start at -1 so the first step logs);
        #       create_timer(0.05, self._tick).
        pass

    def _tick(self):
        # TODO: the step/target math above, log on change, publish the message.
        #       Read the parameters here (not once in __init__) so you can change
        #       them live:  ros2 param set /steering_sweep amplitude_deg 12.0
        pass


def main(args=None):
    rclpy.init(args=args)
    node = SteeringSweep()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

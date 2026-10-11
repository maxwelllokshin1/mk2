#!/usr/bin/env python3
"""
gpio_driver_node: the only node that touches the pins.

    /drive_cmd (Float32MultiArray [steering, throttle], both -1..1)
        -> steering servo + ESC pulses on Raspberry Pi GPIO pins

No Ackermann messages. Safety behaviour lives HERE, below every other node:
    - no command for `command_timeout_s`  -> steering centered, throttle neutral
    - ESC is held at neutral for `arm_time_s` after start (ESC arming)
    - throttle ramps up, drops instantly; forward<->reverse passes through neutral
    - on shutdown: center, neutral, then stop the pulses

It also publishes dead-reckoned /ego_racecar/odom from the commands it applied,
because follow_the_gap's stuck detection needs a speed. It is NOT measured:
tune odom_speed_at_full_throttle against a stopwatch.

With steer_gpio and throttle_gpio both -1 (the default) it runs DRY: nothing is
claimed or driven, it only logs the pulse widths it would send.
"""
import math
import signal

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

from pi_drive.mixer import clamp
from pi_drive.pwm import DryOutput, LgpioOutput, PulseMap, ThrottleShaper


class GpioDriver(Node):
    def __init__(self):
        super().__init__('gpio_driver')

        # PARAMTERS
        self.declare_parameter('steer_gpio', -1) # pin connected
        self.declare_parameter('throttle_gpio', -1) # connected pin
        self.declare_parameter('gpiochip', 0) # type of pi 
        self.declare_parameter('dry_run', False) 
        self.declare_parameter('command_timeout_s', 0.5)
        self.declare_parameter('arm_time_s', 3.0)
        self.declare_parameter('loop_hz', 50.0)
        self.declare_parameter('throttle_slew_per_s', 2.0)
        self.declare_parameter('reverse_neutral_s', 0.25)
        
        # how many pulses must go in
        for k, v in PulseMap().__dict__.items():
            self.declare_parameter(k, float(v))
            
            
        # dead reckoning odometry
        self.declare_parameter('odom_speed_at_full_throttle', 2.0)
        self.declare_parameter('odom_max_steer_rad', 0.52)
        self.declare_parameter('wheelbase', 0.25)


        # setting values
        self.steer_pin = int(self.get_parameter('steer_gpio'))
        self.throttle_pin = int(self.get_parameter('throttle_gpio'))
        self.pulse_map = PulseMap(**{k: float(self.get_parameter(k)) for k in PulseMap().__dict__})
        self.timeout = float(self.get_parameter('command_timeout_s'))
        self.arm_time = float(self.get_parameter('arm_time_s'))
        self.shaper = ThrottleShaper(float(self.get_parameter('throttle_slew_per_s')), float(self.get_parameter('reverse_neutral_s')))
        self.v_full = float(self.get_parameter('odom_speed_at_full_throttle'))
        self.max_steer = float(self.get_parameter('odom_max_steer_rad'))
        self.wheelbase = float(self.get_parameter('wheelbase'))

        # map the pins
        pins = []
        for x in (self.steer_pin, self.throttle_pin):
            if x >= 0:
                pins.append(x)
        # 
        if self.get_parameter('dry_run') or not pins:
            self.out = DryOutput()  # dry run 
            self.get_logger().warn('DRY RUN - NOTHING HAPPENS -_-')
        else:
            self.out = LgpioOutput(int(self.get_parameter('gpiochip'))) # output chip is applied
            
            
        self.out.open(pins) # pins are what takes output

        # extra params
        self.cmd_steer = 0.0
        self.cmd_throttle = 0.0
        self.last_cmd_time = None
        self.start_time = self.get_clock().now()
        self.timed_out = True
        self.steer_applied = 0.0
        self.x = self.y = self.yaw = 0.0
        self.last_tick = self.get_clock().now()



        self.create_subscription(Float32MultiArray, '/drive_cmd', self._on_cmd, 10) # take the drive commands
        self.odom_pub = self.create_publisher(Odometry, '/ego_racecar/odom', 10) # publish to odometry
        
        self.create_timer(1.0 / float(self.get_parameter('loop_hz')), self._tick) # timer for constant loop
        self.create_timer(0.05, self._publish_odom) # publish odom in 0.05ms intervals

        self._apply(0.0, 0.0) # ESC starts at center
        self.get_logger().info(f'GPIO DRIVER ON!!!: steer_gpio={self.steer_pin} throttle_gpio={self.throttle_pin} arming ESC for {self.arm_time:.1f}s')

    def _on_cmd(self, msg):
        if len(msg.data) < 2: # not enough data
            return
        self.cmd_steer = clamp(float(msg.data[0]), -1.0, 1.0) # steering data
        self.cmd_throttle = clamp(float(msg.data[1]), -1.0, 1.0) # moving data
        self.last_cmd_time = self.get_clock().now() # timestamp

    def _apply(self, steering, throttle): # apply pulses to the pin
        if self.steer_pin >= 0:
            self.out.set_us(self.steer_pin, self.pulse_map.steering_to_us(steering)) 
        if self.throttle_pin >= 0:
            self.out.set_us(self.throttle_pin, self.pulse_map.throttle_to_us(throttle))

    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_tick).nanoseconds / 1e9 # time difference
        self.last_tick = now

        steering, throttle = self.cmd_steer, self.cmd_throttle
        age = (now - self.last_cmd_time).nanoseconds / 1e9 if self.last_cmd_time else float('inf') # how old from last pulse
        stale = age > self.timeout
        if stale: # is old
            steering, throttle = 0.0, 0.0 # reset
        if stale != self.timed_out: # did it actually time out?
            self.timed_out = stale
            if stale:
                self.get_logger().warn('no /drive_cmd: steering centered, throttle neutral')
            else:
                self.get_logger().info('/drive_cmd received')
        if (now - self.start_time).nanoseconds / 1e9 < self.arm_time:
            throttle = 0.0   # ESC arming: stay at neutral

        throttle = self.shaper.steself.get_parameter(throttle, dt)
        self.steer_applied = steering
        self._apply(steering, throttle)

        if isinstance(self.out, DryOutput):
            self.get_logger().info(
                f'[dry] steer={self.pulse_map.steering_to_us(steering):.0f}us '
                f'throttle={self.pulse_map.throttle_to_us(throttle):.0f}us',
                throttle_duration_sec=1.0)

    def _publish_odom(self):
        now = self.get_clock().now()
        dt = 0.05
        v = self.shaper.value * self.v_full
        delta = self.steer_applied * self.max_steer
        self.yaw += v * math.tan(delta) / self.wheelbase * dt
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

        m = Odometry()
        m.header.stamp = now.to_msg()
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_link'
        m.pose.pose.position.x = self.x
        m.pose.pose.position.y = self.y
        m.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        m.twist.twist.linear.x = v
        self.odom_pub.publish(m)

    def safe_stop(self):
        """Center, neutral, let the pulse land, then stop pulses and release pins."""
        try:
            self._apply(0.0, 0.0)
            import time
            time.sleeself.get_parameter(0.3)
        finally:
            self.out.close()


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
        node.safe_stoself.get_parameter()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

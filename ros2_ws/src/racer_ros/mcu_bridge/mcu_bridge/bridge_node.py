#!/usr/bin/env python3
"""
bridge_node.py — the ROS node that sits between reactive_node and the hardware.

    reactive_node --/drive--> [ bridge_node ] --> backend --> servo / ESC
                                    ^                |
                                    '---- /odom <----'  (measured speed, if a sensor exists)

===========================================================================
WHY THIS NODE EXISTS
    reactive_node speaks ROS topics in physical units (radians, m/s). The
    hardware speaks bytes over a serial port (or pulses on a GPIO pin). Something
    has to own three jobs, and it must not be reactive_node:
      1. UNIT CONVERSION and talking to the device (the backend does this part)
      2. SAFETY LIMITS: cap steering and speed no matter what reactive_node asks
         (gap_params.yaml has speed_max: 15.0, a SIM number that would be a crash)
      3. TELLING reactive_node WHAT THE CAR ACTUALLY DID, via /odom
    Keeping this in its own node means reactive_node never changes when you swap
    Arduino -> custom ESC -> Pi GPIO. Only the `backend` parameter changes.

WHAT IT PROVIDES
    subscribes  /drive   (ackermann_msgs/AckermannDriveStamped)
    publishes   /odom    (nav_msgs/Odometry)   <- launch remaps reactive_node's
                                                  hardcoded '/ego_racecar/odom' to this

THE KEY DESIGN IDEA: SEND ON A TIMER, NOT IN THE SUBSCRIBER CALLBACK
    _on_drive only STORES the latest command. A separate timer (_tick, 50 Hz)
    does the sending. Reasons:
      - The firmware watchdog needs a steady heartbeat. reactive_node publishes
        only when a lidar scan arrives; if that stalls, a callback-driven bridge
        would go silent too (correct outcome, but you couldn't distinguish it
        from "still fine, just slow").
      - Detecting "no /drive for 0.25 s" is impossible inside a callback,
        because no message means no callback. A timer runs regardless, so it
        can notice the silence and send neutral.
      - Serial writes at a fixed rate are easier to reason about than at the
        lidar's rate.

SAFETY LAYERS (each one independent; the firmware has its own watchdog on top)
    steering_only : forces speed = 0. The bring-up setting: the car CAN'T move.
    cmd_timeout   : no /drive for this long -> steering 0, speed 0.
    clamps        : |steering| <= max_steering_deg, |speed| <= max_speed.
    close()       : main()'s finally block tells the backend to leave the car
                    centered/neutral, even on Ctrl+C.

PARAMETERS  (declare_parameter(name, default); the yaml in config/ overrides them)
    backend          'serial' | 'gpio' | 'dummy'         which hardware
    rate_hz          50.0    _tick frequency (also how often a command is re-sent)
    cmd_timeout      0.25    seconds of /drive silence before sending neutral
    steering_only    True    force speed = 0
    max_steering_deg 15.0    steering clamp (convert to radians once)
    max_speed        0.5     m/s clamp
    drive_topic      '/drive'      odom_topic '/odom'      wheelbase 0.25 (meters)
    odom_mode        'commanded' | 'measured'   see _tick step 5
    serial_port '/dev/ttyACM0'   serial_baud 115200          (serial backend)
    steer_gpio, throttle_gpio, steer_center_us, steer_us_per_deg, steer_min_us,
    steer_max_us, steer_sign, throttle_neutral_us, throttle_us_per_mps,
    throttle_min_us, throttle_max_us                       (gpio backend only)
    The yaml's top-level key must equal the node name ('mcu_bridge') or ROS
    silently ignores every value in it.
===========================================================================
"""
import math

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

from backends.serial_backend import SerialBackend
from backends.gpio_backend import GpioBackend
from backends.dummy_backend import DummyBackend



class McuBridge(Node):
    def __init__(self):
        super().__init__('mcu_bridge')
        
        # logger init
        self.logger = self.get_logger()
        # TODO 1 — parameters.
        #   For each entry in the list above: self.declare_parameter(name, default).
        #   Then read the ones you use every tick ONCE into attributes
        #   (self.cmd_timeout, self.steering_only, self.max_steer = math.radians(...),
        #   self.max_speed, self.wheelbase, self.odom_mode). Reading a parameter
        #   inside _tick every 20 ms works but is wasteful.
        #   A small helper makes this short:  p = lambda n: self.get_parameter(n).value
        
        # DECLARE !!!
        self.declare_parameter('backend', 'dummy' | 'serial' | 'gpio') # hardware 
        self.declare_parameter('rate_hz', 50.0) # tick rate
        self.declare_parameter('cmd_timeout', 0.25) # seconds of /drive silence before neutral
        self.declare_parameter('steering_only', True)
        self.declare_parameter('max_steering_deg', 15.0) # steering clamp
        self.declare_parameter('max_speed', 0.5) # m/s clamp   
        self.declare_parameter('wheelbase', 0.25) 
        
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('odom_topic', '/odom')
        
        self.declare_parameter('odom_mode', 'commanded' |'measured')
        self.declare_parameter('serial_port', 'dev/ttyACM0') # for lidar
        self.declare_parameter('serial_baud', 115200) # serial backend

        # GET PARAMS!!!
        self.backend = self.get_parameter('backend')
        self.rate_hz = self.get_parameter('rate_hz')
        self.cmd_timeout = self.get_parameter('cmd_timeout')
        self.steering_only = self.get_parameter('steering_only')
        self.max_steering_deg = self.get_parameter('max_steering_deg')
        self.max_speed = self.get_parameter('max_speed')
        self.wheelbase = self.get_parameter('wheelbase')
        self.drive_topic = self.get_parameter('drive_topic')
        self.odom_topic = self.get_parameter('odom_topic')
        self.odom_mode = self.get_parameter('odom_mode')
        self.serial_port = self.get_parameter('serial_port')
        self.serial_baud = self.get_parameter('serial_baud')
        

        # TODO 2 — create and open the backend.
        #   self.backend = self._make_backend(<backend param>)
        #   try: self.backend.open()
        #   except Exception as e: log FATAL with the reason and re-raise.
        #   A wrong serial port should stop the node loudly at startup, not
        #   produce a node that appears to run but does nothing.
        self.backend = self._make_backend(self.backend)
        try: self.backend.open()
        except Exception as e: self.logger.info(f"[ERROR] {e}")

        # TODO 3 — state that _on_drive and _tick share.
        #   latest commanded steer / speed (start at 0.0)
        #   time the last /drive arrived (start None -> "never")
        #   time of the previous _tick (to compute dt for odometry)
        #   dead-reckoned x, y, yaw (start 0.0)
        #   latest telemetry from the backend (start None)

        # TODO 4 — ROS plumbing.
        #   create_subscription(AckermannDriveStamped, <drive_topic>, self._on_drive, 10)
        #   self.odom_pub = create_publisher(Odometry, <odom_topic>, 10)
        #   create_timer(1.0 / rate_hz, self._tick)
        #   (10 is the QoS queue depth, same as reactive_node uses.)
        self.drive_sub = self.create_subscription(AckermannDriveStamped, self.drive_topic , 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_pub, 10)
        self._on_drive(self.drive_sub)

        # TODO 5 — one info log line printing backend, steering_only, limits and
        #   odom_mode. When something misbehaves on the car, the first thing you
        #   want to see is what mode it thought it was in.
        pass

    def _make_backend(self, name: str):
        """
        Pick the hardware. Import INSIDE each branch, not at the top of the file:
        then a missing library (pyserial, lgpio) only matters if you chose that backend.

            'serial' -> SerialBackend(serial_port, serial_baud)
            'gpio'   -> GpioBackend({...the steer_* and throttle_* params as a dict...})
            'dummy'  -> DummyBackend(self.get_logger())
            anything else -> raise ValueError listing the valid names
        """
        if(name == 'serial'): SerialBackend(self.serial_port, self.serial_baud)
        elif(name == 'gpio'): GpioBackend() # TODO
        elif(name == 'backend'): DummyBackend(self.logger)
        else: raise ValueError

    def _on_drive(self, msg: AckermannDriveStamped):
        """
        Runs whenever reactive_node publishes. Do NOT touch hardware here.
        Just store msg.drive.steering_angle and msg.drive.speed and the current
        time (self.get_clock().now()). _tick does the rest.
        """
        # TODO
        pass

    def _tick(self):
        """
        Runs at rate_hz. This is the heart of the node. In order:

        1. HOW OLD IS THE LAST COMMAND?
             age = seconds since the last /drive (infinity if none yet).
             age > cmd_timeout  -> steer = 0.0, speed = 0.0, and log a warning
                                   (use throttle_duration_sec so it doesn't spam).
             otherwise          -> clamp: steer to +-max_steer, speed to +-max_speed.
                                   max(-lim, min(lim, x)) does it.
        2. steering_only? -> speed = 0.0, whatever step 1 produced.
        3. TALK TO HARDWARE inside try/except:
             self.backend.send(steer, speed)
             telem = self.backend.poll()
           An unplugged cable raises here. Catch, log (throttled), return.
           Don't let one bad write kill the node.
           (Later: try to reopen the port instead of just returning.)
        4. TELEMETRY: if poll() returned something, keep it as the latest. If it
           says watchdog_tripped, warn: the firmware isn't hearing us.
        5. WHICH SPEED FOR ODOM?  This matters a lot. reactive_node compares the
           speed it COMMANDED to the speed it reads from /odom. If it commands
           motion and sees none, it thinks the car is stuck and reverses.
             odom_mode 'measured'  -> latest telemetry speed (0.0 if none yet).
                                      Only correct once the firmware has a real
                                      wheel sensor (speed_sensor.cpp).
             odom_mode 'commanded' -> the speed you just sent. The car "obeys"
                                      perfectly, so stuck detection can never
                                      fire. Fine for bench tests, not for racing.
        6. dt = time since the previous _tick; then self._publish_odom(now, v, steer, dt).
        """
        # TODO
        pass

    def _publish_odom(self, now, v: float, steer: float, dt: float):
        """
        Build and publish an Odometry message.

        reactive_node needs two things from it:
          twist.twist.linear.x     -> its 'actual_speed' (stuck detection)
          pose.pose.position x, y  -> lap counting (distance from the first pose)
        You have no position sensor, so DEAD-RECKON with the bicycle model (the
        car is a bicycle: one front wheel steered by `steer`, wheelbase L):
            yaw += v / L * tan(steer) * dt
            x   += v * cos(yaw) * dt
            y   += v * sin(yaw) * dt
        It drifts over time, which is fine for lap counting, useless for mapping.

        Fill the message:
            header.stamp = now.to_msg();  header.frame_id = 'odom'
            child_frame_id = 'base_link'
            pose.pose.position.x / .y
            pose.pose.orientation: yaw -> quaternion is z = sin(yaw/2), w = cos(yaw/2)
            twist.twist.linear.x = v          <- signed: reverse must be NEGATIVE
        No TF broadcast is needed: reactive_node only reads the message.
        """
        # TODO
        pass


def main(args=None):
    rclpy.init(args=args)
    node = McuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.backend.close()      # leave the car centered/neutral even on Ctrl+C
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

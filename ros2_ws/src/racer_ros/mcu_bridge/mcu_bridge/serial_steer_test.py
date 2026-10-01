#!/usr/bin/env python3
"""
serial_steer_test.py — talk to the firmware directly, with NO ROS involved.

WHY IT EXISTS
    When something doesn't work on the first hardware test, you need to know
    which half is broken. This tool removes ROS, the bridge node and the launch
    files from the picture: if it can steer the wheels, the wiring, the
    firmware and protocol.py are good, and any remaining problem is ROS-side.
    Use it FIRST, before the ROS steering test. It's also the tool for
    calibrating: type an angle, watch the wheels, adjust config.h.

    Run it from the package folder so `mcu_bridge.protocol` imports without a
    colcon build (protocol.py imports no ROS on purpose):
        cd ros2_ws/src/racer_ros/mcu_bridge
        python3 -m mcu_bridge.serial_steer_test /dev/ttyACM0

BEHAVIOUR TO BUILD  (compare scripts/real/motor_test.py: same idea, worth a look)
    Two things happen at once, so two threads:

    BACKGROUND THREAD, every 50 ms:
        write encode_command(radians(current_deg), 0.0, True)
        read + parse telemetry; print what the board reports back
        (steering, speed, watchdog flag), rate-limited to ~2 lines/second
      It must keep re-sending even when you type nothing: the firmware watchdog
      re-centers the wheels after 200 ms without a valid command, so a "send
      once" tool would appear to do nothing.

    MAIN THREAD, an input() loop:
        a number  -> set the current steering (degrees, + = left), clamped to
                     +-MAX_TEST_DEG so a typo like "150" can't slam the linkage
        q         -> quit
      Share the current value between threads through a small dict or a
      threading.Event/Lock. (One float assigned by one thread and read by the
      other is fine in Python.)

    STARTUP: serial.Serial(port, 115200, timeout=0), then sleep ~2 s (the
    Arduino resets when the port opens), then reset_input_buffer().

    SHUTDOWN (finally): set the angle to 0, give the thread ~0.2 s to actually
    send it, stop the thread (a `run` flag + join), send ONE enable=False
    command, close the port.

WHAT TO TRY WHEN IT WORKS
    5, -5, 0, then bigger values up to 15; check the direction, check the
    wheels stop at the clamp; then Ctrl+C mid-command and confirm the wheels
    re-center on their own (that proves the watchdog).
"""
import sys
import threading
import time

MAX_TEST_DEG = 15.0


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    # TODO: everything above. Imports you'll want:
    #   import math, serial
    #   from mcu_bridge.protocol import FrameParser, TYPE_TELEM, decode_telemetry, encode_command
    print(f'TODO: talk to {port}')


if __name__ == '__main__':
    main()

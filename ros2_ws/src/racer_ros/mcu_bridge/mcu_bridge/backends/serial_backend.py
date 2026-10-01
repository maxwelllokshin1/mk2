"""
backends/serial_backend.py — Arduino (now) or your own ESC board (later), over serial.

WHY IT EXISTS / WHAT IT PROVIDES
    Implements the Backend contract (see base.py) by speaking the byte protocol
    in protocol.py to the firmware in mcu_ws/. It's the only class that knows a
    serial port exists. It turns (radians, m/s) into a command frame, and turns
    incoming telemetry frames into Telemetry objects.
    It is also the only backend with feedback, so it's the only one that can
    make odom_mode: measured work.

HOW IT WORKS
    send(): every tick, encode_command(...) and write the bytes. It writes even
            when nothing changed, because that repeated frame IS the heartbeat
            the firmware watchdog listens for.
    poll(): read whatever bytes have arrived (non-blocking), feed them to a
            FrameParser, and return the newest telemetry frame, if any.

THINGS THAT WILL BITE YOU
    - NON-BLOCKING READS: open the port with timeout=0. Then read() returns
      immediately with whatever is there (maybe nothing). A blocking read would
      freeze bridge_node's whole timer.
    - THE ARDUINO RESETS WHEN YOU OPEN THE PORT: opening toggles the DTR line,
      which reboots an Uno/Nano for ~1-2 s. Bytes sent during the reboot go
      nowhere, and the firmware starts in its failsafe state. So after
      opening: sleep ~2 s, then reset_input_buffer() to discard boot-time
      garbage. (A custom board may not do this; keep it a parameter.)
    - PARTIAL FRAMES: read(256) may return half a frame. That's why a
      FrameParser with an internal buffer exists; keep ONE parser for the life
      of the backend (a new one per call would forget the first half).
    - The device name changes (ttyACM0 vs ttyACM1). Prefer the stable
      /dev/serial/by-id/... path in the yaml.

pyserial API you need (import serial, INSIDE open() so other backends don't
require it):
    serial.Serial(port, baud, timeout=0)   open      .write(bytes)   .read(n)
    .reset_input_buffer()   .flush()   .close()
"""
from typing import Optional

from mcu_bridge.backends.base import Backend
from mcu_bridge.protocol import Telemetry


class SerialBackend(Backend):
    def __init__(self, port: str, baud: int, boot_delay: float = 2.0):
        # TODO: store the arguments; self._ser = None; self._parser = FrameParser().
        raise NotImplementedError

    def open(self) -> None:
        # TODO: open non-blocking, wait boot_delay (the reset above), drop
        #       boot-time bytes.
        raise NotImplementedError

    def send(self, steering_rad: float, speed_mps: float) -> None:
        # TODO: write(encode_command(steering_rad, speed_mps, enable=True))
        raise NotImplementedError

    def poll(self) -> Optional[Telemetry]:
        # TODO: read up to 256 bytes, feed the parser; for each TYPE_TELEM frame
        #       decode_telemetry(payload); return the LAST one, or None if none.
        raise NotImplementedError

    def close(self) -> None:
        # TODO: in try/finally: best-effort write an enable=False, zero command
        #       and flush, so the firmware goes to failsafe immediately rather
        #       than waiting for its watchdog; the finally closes the port.
        #       Handle "never opened" (self._ser is None) without raising.
        raise NotImplementedError

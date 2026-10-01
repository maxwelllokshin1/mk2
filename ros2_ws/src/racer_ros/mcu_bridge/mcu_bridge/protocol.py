"""
protocol.py — Pi-side twin of mcu_ws/src/protocol.cpp.

WHY IT'S ITS OWN FILE
    No ROS imports here on purpose. bridge_node, serial_steer_test.py and any
    future unit test all share it, and it must stay byte-for-byte compatible
    with the firmware. Read the long comment in protocol.cpp first: it explains
    the frame, the CRC and the state machine. This file is the same thing in Python.

FRAME (both directions), little-endian:
    0xAA 0x55 | type (1) | len (1) | payload (len) | crc8 (1)
    crc8 covers type, len and payload (NOT the sync bytes).

    TYPE_CMD   (Pi -> MCU)  payload '<hhB': steering_cdeg, speed_mmps, flags(bit0=enable)
    TYPE_TELEM (MCU -> Pi)  payload '<hhB': steering_cdeg, speed_mmps, status(bit0=watchdog)
    cdeg = centi-degrees (1234 = 12.34 deg); mmps = mm/s.

TEST VECTORS (make your implementation reproduce these exactly; they were
generated from a known-good version and the firmware is checked against them):
    encode_command(radians(12.34), 0.5, True).hex() == 'aa550105d204f4010176'
    crc8(bytes([0x01,0x05,0xD2,0x04,0xF4,0x01,0x01])) == 0x76
    FrameParser().feed(bytes.fromhex('aa550205d204000000c5'))
        -> [(TYPE_TELEM, payload)] that decodes to steer 12.34 deg, speed 0, watchdog False
    a frame with one byte corrupted must yield NO frames, and the parser must
    still find the next good frame after it (resync).
"""
import math
import struct
from dataclasses import dataclass

SYNC = b'\xAA\x55'
MAX_PAYLOAD = 16

TYPE_CMD = 0x01
TYPE_TELEM = 0x02

CMD_FMT = '<hhB'
TELEM_FMT = '<hhB'

FLAG_ENABLE = 0x01
STATUS_WATCHDOG = 0x01


def crc8(data: bytes) -> int:
    """CRC-8, poly 0x07, init 0, MSB first. Same algorithm as protocol.cpp PART 1."""
    # TODO
    raise NotImplementedError


def encode_frame(msg_type: int, payload: bytes) -> bytes:
    """SYNC + type + len + payload + crc8(type, len, payload)."""
    # TODO
    raise NotImplementedError


def encode_command(steering_rad: float, speed_mps: float, enable: bool = True) -> bytes:
    """
    Convert to the wire units (rad -> centi-degrees, m/s -> mm/s), round to int,
    clamp to the int16 range (-32768..32767) so struct.pack can't raise, pack
    with CMD_FMT, then encode_frame(TYPE_CMD, ...).
    """
    # TODO
    raise NotImplementedError


@dataclass
class Telemetry:
    steering_rad: float
    speed_mps: float          # signed: + forward, - reverse
    watchdog_tripped: bool


def decode_telemetry(payload: bytes) -> Telemetry:
    """struct.unpack with TELEM_FMT, then convert back to rad / m/s / bool."""
    # TODO
    raise NotImplementedError


class FrameParser:
    """
    Feed raw bytes in (any chunking), get complete valid (type, payload) frames out.

    Same job as the C++ state machine, but on the Pi you have a bytearray
    buffer, so it's easier: keep the unread bytes in self._buf; on each feed()
    append, then loop:
        1. find SYNC in the buffer; discard everything before it. If not found,
           keep only the last byte (it might be a lone 0xAA) and stop.
        2. need at least 4 bytes to read len; need 4 + len + 1 for a full frame.
           If len > MAX_PAYLOAD, drop one byte and rescan.
        3. if the crc matches: emit (type, payload) and consume the frame.
           If not: drop ONE byte (not the whole frame) and rescan, so a
           corrupted frame can't swallow a good one that starts inside it.
    """

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes):
        # TODO
        raise NotImplementedError

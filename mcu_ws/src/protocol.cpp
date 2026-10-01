// protocol.cpp — how the Pi and this board talk over the serial cable.
//
// ---------------------------------------------------------------------------
// WHY A PROTOCOL AT ALL?
//   A serial port is just a stream of bytes. It has no notion of "message".
//   Bytes can arrive in pieces (half a command now, half in 2 ms), the board
//   can be reset mid-message, and electrical noise can flip bits. So both
//   sides need agreed rules for: (1) where a message starts, (2) how long it
//   is, (3) how to detect corruption. Without them you'll occasionally steer
//   to a garbage angle, which on a car is a crash.
//
// WHAT THIS FILE PROVIDES (see protocol.h)
//   protocolBegin()          open the port
//   protocolPoll(cmd)        "did a full valid command arrive?"
//   protocolSendTelemetry()  build and send a frame to the Pi
//
// THE FRAME (same layout both directions; the Python twin is
// ros2_ws/src/racer_ros/mcu_bridge/mcu_bridge/protocol.py — they MUST agree):
//
//   byte:   0     1     2      3     4 ... 4+len-1    4+len
//         +-----+-----+------+-----+---------------+--------+
//         | 0xAA| 0x55| type | len |    payload    |  crc8  |
//         +-----+-----+------+-----+---------------+--------+
//          \_sync_/     \____ covered by crc8 ____________/
//
//   type 0x01 = COMMAND   (Pi -> board), len = 5:
//       int16 steering_cdeg | int16 speed_mmps | uint8 flags (bit0 = enable)
//   type 0x02 = TELEMETRY (board -> Pi), len = 5:
//       int16 steering_cdeg | int16 speed_mmps | uint8 status (bit0 = watchdog tripped)
//   int16 values are LITTLE-ENDIAN: low byte first, then high byte.
//
// ---------------------------------------------------------------------------
// PART 1: CRC-8 (protocolCrc8Update)
//   A checksum: the sender computes a number from the bytes, appends it, and
//   the receiver recomputes it. If they differ, throw the frame away.
//   Algorithm (poly 0x07, start value 0, most-significant-bit first), for each
//   byte b fed in:
//       crc = crc XOR b
//       repeat 8 times:
//           if the top bit of crc (crc & 0x80) is set:  crc = (crc << 1) XOR 0x07
//           else:                                       crc = (crc << 1)
//           (keep crc to 8 bits: cast to uint8_t)
//   Feed it: type, then len, then every payload byte. Not the sync bytes.
//
//   TEST IT BEFORE ANYTHING ELSE. Known-good values (computed by the Python
//   side, which is tested):
//       crc8 over bytes {0x01,0x05, 0xD2,0x04, 0xF4,0x01, 0x01} == 0x76
//       crc8 over bytes {0x02,0x05, 0xD2,0x04, 0x00,0x00, 0x00} == 0xC5
//   If yours differs, the bug is in the shift/XOR/8-bit cast. Fix it now;
//   every later problem will look like a mystery otherwise.
//
// PART 2: RECEIVING (protocolPoll) — a state machine
//   Serial gives you one byte at a time (Serial.available() / Serial.read()).
//   You can't assume a whole frame is there, so remember where you are:
//
//     WAIT_SYNC0 --0xAA--> WAIT_SYNC1 --0x55--> WAIT_TYPE --> WAIT_LEN
//         ^  (anything else: stay/return here; if you see 0xAA again in
//         |   WAIT_SYNC1, stay in WAIT_SYNC1)
//         |                              |
//         |   len > MAX_PAYLOAD? give up and go back to WAIT_SYNC0
//         |                              v
//         +---- after CRC check <---- WAIT_CRC <--- WAIT_PAYLOAD (collect `len` bytes,
//                                                    skipped if len == 0)
//   At WAIT_CRC: recompute the crc over type, len, payload. Equal -> the frame
//   is good: if type == 0x01 and len == 5, decode it into a Command. Not equal
//   -> silently drop it. Either way, go back to WAIT_SYNC0.
//   The state variables (current state, type, len, index, payload buffer) must
//   be `static` file-level variables so they survive between calls.
//
//   Decoding little-endian int16 from payload p[]:
//       steerCdeg = (int16_t)(p[0] | (p[1] << 8));
//   flags: enable = (p[4] & 0x01) != 0.
//
//   protocolPoll must be NON-BLOCKING: loop `while (Serial.available())`, feed
//   each byte to the state machine, and return. Never wait for bytes.
//   If several frames arrive in one call, keep only the newest.
//
// PART 3: SENDING (protocolSendTelemetry)
//   Build the 5-byte payload (steering low, steering high, speed low, speed
//   high, status), compute the crc over type/len/payload, then Serial.write
//   the bytes in order: 0xAA, 0x55, type, len, payload..., crc.
//   TEST VECTOR: steer = 1234, speed = 0, watchdog = false must produce
//       AA 55 02 05 D2 04 00 00 00 C5
//   and watchdog = true with steer 0 must produce
//       AA 55 02 05 00 00 00 00 01 33
//
// GOTCHA: Serial is this protocol's channel, so NEVER Serial.print() text
//   anywhere in the firmware. Use the LED for debugging (see README).
// ---------------------------------------------------------------------------

#include "protocol.h"
#include "config.h"

// TODO: constants for SYNC0 (0xAA), SYNC1 (0x55), TYPE_COMMAND, TYPE_TELEMETRY,
//       MAX_PAYLOAD (16 is plenty), FLAG_ENABLE, STATUS_WATCHDOG.
// TODO: the state-machine enum and the `static` state variables.

void protocolBegin() {
  // TODO: Serial.begin(BAUD);
}

uint8_t protocolCrc8Update(uint8_t crc, uint8_t byte) {
  // TODO: the algorithm from PART 1. Return the new crc.
  (void)byte;
  return crc;
}

bool protocolPoll(Command& out) {
  // TODO: PART 2. Return true only if a new valid command completed.
  (void)out;
  return false;
}

void protocolSendTelemetry(int16_t steerCdeg, int16_t speedMmps, bool watchdogTripped) {
  // TODO: PART 3.
  (void)steerCdeg;
  (void)speedMmps;
  (void)watchdogTripped;
}

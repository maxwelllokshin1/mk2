// protocol.h — the public face of protocol.cpp. Implementation notes live there.
#pragma once
#include <Arduino.h>

// One decoded command from the Pi, in integer units so no floats cross the wire.
struct Command {
  int16_t steerCdeg;   // steering in CENTI-degrees (1234 = 12.34 deg), + = left
  int16_t speedMmps;   // speed in mm/s, + = forward
  bool    enable;      // false = Pi is asking us to stay in failsafe
};

// Call once from setup(): opens the serial port (Serial.begin(BAUD)).
void protocolBegin();

// Call every loop(): consumes whatever bytes have arrived. Returns true if at
// least one COMPLETE, CRC-valid command frame finished during this call, and
// copies the newest one into `out`. Must never block.
bool protocolPoll(Command& out);

// Send one telemetry frame back to the Pi.
void protocolSendTelemetry(int16_t steerCdeg, int16_t speedMmps, bool watchdogTripped);

// Exposed so you can unit-test it (see the test vectors in protocol.cpp).
uint8_t protocolCrc8Update(uint8_t crc, uint8_t byte);

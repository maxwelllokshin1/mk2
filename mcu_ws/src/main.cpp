// main.cpp — the conductor. It contains no hardware logic itself; it calls the
// other modules in the right order. If main.cpp is getting long, something
// belongs in a module instead.
//
// ---------------------------------------------------------------------------
// HOW AN ARDUINO PROGRAM RUNS
//   There is no operating system and no threads. After power-up or reset the
//   chip runs setup() ONCE, then calls loop() over and over, as fast as it
//   can (thousands of times per second). Everything you do must fit inside
//   one quick pass of loop().
//
//   RULE 1: NEVER delay(). It freezes everything, including reading serial
//           and the watchdog check. If you need "every 20 ms", remember
//           when you last did it and compare against millis():
//               if ((uint32_t)(millis() - lastMs) >= PERIOD) { lastMs = millis(); ... }
//   RULE 2: NEVER Serial.print() debug text. Serial is the protocol channel
//           (see README). Blink LED_BUILTIN to show state instead.
//   RULE 3: Servo pulses are generated in the background by a hardware timer,
//           so a slow loop() doesn't make the servo jitter, but a slow loop()
//           does make you miss serial bytes (the Uno's receive buffer is
//           only 64 bytes).
//
// WHAT setup() MUST DO, in this order
//   1. steeringBegin()     wheels go to center right away, before the Pi
//                          has said anything
//   2. throttleBegin()     (does nothing while ENABLE_THROTTLE is 0)
//   3. speedSensorBegin()
//   4. protocolBegin()     opens the serial port
//   5. pinMode(LED_BUILTIN, OUTPUT)
//
// WHAT loop() MUST DO, every pass, in this order
//   1. READ:     Command cmd;
//                if (protocolPoll(cmd)) { latest = cmd; watchdogFeed(); }
//                (`latest` is a file-level static so it persists between passes)
//   2. DECIDE:   bool failsafe = watchdogTripped() || !latest.enable;
//   3. ACT:      if (failsafe) { steeringCenter(); throttleNeutral(); }
//                else          { steeringSetCdeg(latest.steerCdeg);
//                                throttleSetMmps(latest.speedMmps); }
//                The failsafe branch is the whole point of the watchdog:
//                whatever happened upstream, the car ends up straight and stopped.
//   4. REPORT:   every TELEM_PERIOD_MS (millis() timer, Rule 1):
//                protocolSendTelemetry(steering actually applied — 0 if failsafe,
//                                      speedSensorMmps(),
//                                      watchdogTripped());
//   5. INDICATE: LED on when !failsafe, off when in failsafe. One glance at
//                the board tells you whether it's hearing the Pi.
//
// HOW TO TELL IT'S WORKING (in order)
//   - Power on: wheels center, LED off (no Pi yet, watchdog tripped).
//   - Run mcu_bridge's serial_steer_test: LED turns on, typing angles moves the wheels.
//   - Ctrl+C the test tool: within ~200 ms LED off and wheels centered.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include "config.h"
#include "protocol.h"
#include "steering.h"
#include "throttle.h"
#include "speed_sensor.h"
#include "watchdog.h"

// TODO: `static Command latest = {0, 0, false};` and `static uint32_t lastTelemMs = 0;`

void setup() {
  // TODO: the five steps above.
}

void loop() {
  // TODO: the five steps above.
}

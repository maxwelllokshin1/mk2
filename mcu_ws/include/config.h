// config.h — every number you will ever want to tune, in ONE place.
//
// WHY THIS FILE EXISTS
//   Calibration values (servo center, limits, pins, timeouts) are the things
//   you'll change most while testing on the car. If they're scattered through
//   the code as "magic numbers" you'll miss one and lose an hour. If they live
//   here, retuning is a one-file edit and nothing else can drift out of sync.
//
// WHAT GOES IN IT
//   Only constants and #defines. No functions, no variables that change.
//   Use `constexpr` (a compile-time constant with a type) instead of #define
//   for numbers: the compiler checks the type and it shows up in the debugger.
//   Use #define only for the on/off feature switch below, because the code
//   uses `#if ENABLE_THROTTLE` to compile whole sections in or out.
//
// `#pragma once` stops the file being included twice in one build.
#pragma once
#include <Arduino.h>

// ============================ FEATURE SWITCH ============================
// 0 = steering-only bring-up. The throttle pin is never driven, so the car
//     cannot move even if a bug sends a huge speed. Keep the ESC unplugged.
// 1 = throttle enabled. Only flip this after steering works AND the watchdog
//     is proven (stop the Pi -> throttle returns to neutral).
#define ENABLE_THROTTLE 0

// ================================ PINS ==================================
// TODO: choose pins and write them here, e.g.
//   constexpr uint8_t PIN_STEER = 9;
//   constexpr uint8_t PIN_THROTTLE = 10;
//   constexpr uint8_t PIN_SPEED_SENSOR = 2;   // must be an interrupt-capable pin
//                                             // (Uno: pin 2 or 3; check YOUR board)
// The Servo library works on any digital pin, but it takes over a hardware
// timer to make the pulses (Timer1 on the Uno), which disables analogWrite()
// on pins 9 and 10. You aren't using analogWrite, so that's fine.

// ============================== SERIAL/TIMING ===========================
// TODO:
//   constexpr uint32_t BAUD = 115200;        // MUST equal serial_baud in bridge_params.yaml
//   constexpr uint16_t WATCHDOG_MS = 200;    // how long silence is tolerated. The Pi sends
//                                            // at 50 Hz (every 20 ms), so 200 ms = 10 missed
//                                            // frames. Too small: false trips. Too big: the
//                                            // car coasts blindly for longer when the Pi dies.
//   constexpr uint16_t TELEM_PERIOD_MS = 20; // how often to send telemetry back

// ========================= STEERING CALIBRATION =========================
// The servo takes a pulse width in microseconds. You must MEASURE these on
// your car; do not trust the placeholders. Procedure:
//   1. CENTER: send 1500 us. If the wheels aren't straight, nudge until they
//      are (or fix it mechanically at the linkage) and record that value.
//   2. MIN/MAX: from center, step the pulse 25 us at a time toward each side.
//      STOP the moment the linkage touches its mechanical limit or the servo
//      starts to buzz/strain. Back off ~50 us for margin. A servo pushing
//      against a hard stop draws a lot of current and can burn out in minutes.
//   3. US_PER_DEG: measure the actual wheel angle at one extreme (protractor
//      or a phone angle app), then  (extreme_us - CENTER) / measured_degrees.
//   4. SIGN: +1 if positive steering (left) turns the wheels left, else -1.
//      (ROS convention: positive steering_angle = counter-clockwise = LEFT.)
// TODO:
//   constexpr int   STEER_CENTER_US  = 1500;
//   constexpr float STEER_US_PER_DEG = 10.0f;   // placeholder
//   constexpr int   STEER_MIN_US     = 1300;    // placeholder, conservative
//   constexpr int   STEER_MAX_US     = 1700;    // placeholder, conservative
//   constexpr int   STEER_SIGN       = 1;

// ========================= THROTTLE CALIBRATION =========================
// Only used when ENABLE_THROTTLE is 1. Fill in later. Concepts you'll need:
//   - NEUTRAL_US: the pulse at which the motor is stopped (usually ~1500).
//   - The ESC ignores small offsets around neutral (the "deadband"), so very
//     small speeds do nothing; find the smallest pulse that actually turns
//     the wheels.
//   - Keep MIN/MAX tight (e.g. 1450..1550) until you've watched it on a stand.
// TODO (later):
//   constexpr int   THROTTLE_NEUTRAL_US = 1500;
//   constexpr float THROTTLE_US_PER_MPS = 100.0f;   // needs real measurement
//   constexpr int   THROTTLE_MIN_US     = 1450;
//   constexpr int   THROTTLE_MAX_US     = 1550;

// ============================ SPEED SENSOR ==============================
// TODO (later): pulses per wheel revolution and wheel circumference, so
// speed_sensor.cpp can convert "pulses per second" into mm/s.
//   constexpr float PULSES_PER_REV  = 1.0f;     // depends on magnets/encoder slots
//   constexpr float WHEEL_CIRC_MM   = 250.0f;   // measure: diameter * 3.14159

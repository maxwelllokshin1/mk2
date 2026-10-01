// speed_sensor.cpp — how fast is the car ACTUALLY going? LEAVE FOR LATER; until
// you write it, return 0 (the skeleton already does).
//
// ---------------------------------------------------------------------------
// WHY IT MATTERS FOR YOUR SOFTWARE
//   reactive_node compares the speed it commanded against the speed it
//   measured (odom twist.linear.x). If it commands motion and sees none, it
//   decides the car is stuck and starts a reverse recovery. So with no
//   sensor, the Pi has to fake the measurement (odom_mode: commanded in
//   bridge_params.yaml) and that stuck detection can never fire. This file is
//   what makes the real measurement possible.
//
// HOW A WHEEL SENSOR WORKS
//   A hall sensor (or encoder) produces one electrical pulse each time a
//   magnet (or slot) passes it. Count pulses over time:
//       revolutions/s = (pulses/s) / PULSES_PER_REV
//       speed (mm/s)  = revolutions/s * WHEEL_CIRC_MM          (config.h)
//
// HOW YOU COUNT PULSES WITHOUT MISSING ANY: INTERRUPTS
//   loop() might be busy when a pulse arrives, so don't poll the pin. Ask the
//   hardware to call a function the instant the pin changes:
//       attachInterrupt(digitalPinToInterrupt(PIN_SPEED_SENSOR), onPulse, RISING);
//   `onPulse` is an ISR (interrupt service routine), with strict rules:
//       - keep it tiny: just record the time / bump a counter, then return
//       - no Serial, no delay(), no floating point
//       - any variable shared with loop() must be declared `volatile`
//         (tells the compiler it can change behind its back)
//       - when loop() reads a multi-byte shared variable, disable interrupts
//         briefly (noInterrupts(); copy; interrupts();) or it may read half
//         an update — a real bug that shows up randomly.
//   Simple approach: in the ISR store micros() of the latest pulse and the
//   time since the previous one. Speed = distance per pulse / that interval.
//   If no pulse has arrived for a while (say 0.5 s), report speed 0.
//
// DIRECTION (important)
//   A single hall sensor cannot tell forward from backward. The recovery logic
//   checks for NEGATIVE speed while reversing, so you must supply the sign:
//   easiest is to use the sign of the last throttle command (throttle.cpp knows
//   it). A quadrature encoder (two offset channels) measures direction for real.
//
// WHAT TO IMPLEMENT
//   speedSensorBegin(): pinMode(PIN_SPEED_SENSOR, INPUT_PULLUP or INPUT, per
//                       your sensor), attachInterrupt(...).
//   speedSensorMmps():  the conversion above, with the sign, safe against the
//                       volatile-read problem. Clamp to int16 range.
// ---------------------------------------------------------------------------

#include "speed_sensor.h"
#include "config.h"

void speedSensorBegin() {
  // TODO (later)
}

int16_t speedSensorMmps() {
  // TODO (later)
  return 0;
}

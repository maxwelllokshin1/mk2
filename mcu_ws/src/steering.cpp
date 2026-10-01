// steering.cpp — turns "steer 12.34 degrees left" into a servo pulse.
//
// ---------------------------------------------------------------------------
// HOW A HOBBY SERVO WORKS
//   A servo has one signal wire. Every 20 ms (50 Hz) you send it a HIGH pulse;
//   the pulse WIDTH is the command:
//        ~1000 us = one extreme,  ~1500 us = center,  ~2000 us = other extreme
//   It doesn't need a new command each cycle to keep position, but it does
//   need the pulse train to keep running.
//
// WHAT THE Servo LIBRARY DOES FOR YOU
//   #include <Servo.h>   (ships with the Arduino framework; no install)
//       Servo steerServo;                       // one object per servo
//       steerServo.attach(PIN_STEER);           // claims the pin, starts a
//                                               // background timer that emits
//                                               // the 50 Hz pulse train
//       steerServo.writeMicroseconds(1500);     // sets the pulse width
//   Because a hardware timer generates the pulses, they stay steady even
//   while loop() is busy. You only call writeMicroseconds when the target
//   changes (calling it every loop is harmless but wasteful).
//   Prefer writeMicroseconds() over write(angle): write() maps 0-180 to a
//   fixed range you can't calibrate, and you need the exact µs to tune.
//
// THE MATH (values come from config.h; measure them, see the procedure there)
//   us = STEER_CENTER_US + STEER_SIGN * (cdeg / 100.0) * STEER_US_PER_DEG
//   us = constrain(us, STEER_MIN_US, STEER_MAX_US)     <-- the safety clamp
//   The clamp is not optional: it's what stops a bad command from driving the
//   servo into its mechanical stop, where it stalls, overheats and dies. It
//   sits HERE (the last thing before the hardware) so nothing upstream, not
//   the Pi and not a bug in your protocol code, can bypass it.
//   cdeg is centi-degrees (1234 = 12.34 deg); divide by 100.0 (a float!) —
//   dividing by integer 100 would throw away the fraction.
//
// WHAT TO IMPLEMENT
//   steeringBegin():  attach the servo, then IMMEDIATELY write the center pulse.
//                     Do this in setup() so the wheels are straight on boot,
//                     before the Pi has said anything.
//   steeringSetCdeg(): the math above, then writeMicroseconds.
//   steeringCenter():  steeringSetCdeg(0) is fine.
//
// HOW TO KNOW IT WORKS (before any protocol code exists)
//   Call steeringSetCdeg(500) at the end of setup(), upload, and watch. Wheels
//   should turn left ~5 degrees. If they go right, flip STEER_SIGN. Try
//   -500, and larger values, to confirm the clamp holds at the limits.
// ---------------------------------------------------------------------------

#include "steering.h"
#include "config.h"
// TODO: #include <Servo.h>  and declare the `static Servo steerServo;` object.

void steeringBegin() {
  // TODO: attach to PIN_STEER, write the center pulse.
}

void steeringSetCdeg(int16_t cdeg) {
  // TODO: compute microseconds, clamp, writeMicroseconds.
  (void)cdeg;
}

void steeringCenter() {
  // TODO
}

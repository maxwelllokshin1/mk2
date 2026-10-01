// throttle.cpp — turns "drive 1.5 m/s" into an ESC pulse. LEAVE THIS FOR AFTER
// STEERING WORKS. Until then ENABLE_THROTTLE is 0 in config.h and every
// function here does nothing, so the car physically cannot move.
//
// ---------------------------------------------------------------------------
// HOBBY ESC BASICS
//   A hobby ESC takes the same kind of pulse as a servo:
//        1500 us = neutral (stopped),  >1500 = forward,  <1500 = reverse
//   The Servo library drives it the same way as steering.cpp does.
//
// THINGS THAT WILL BITE YOU
//   1. ARMING: most ESCs won't respond until they've seen a neutral pulse for
//      ~1-3 seconds after power-up (they beep when armed). So throttleBegin()
//      must write neutral, and the car should not be commanded to move until
//      that time has passed. If the ESC ignores you, this is the first suspect.
//   2. DEADBAND: pulses close to 1500 do nothing. A tiny "speed" command may
//      not move the car at all. Find the smallest pulse that turns the wheels
//      and account for it in the mapping.
//   3. REVERSE: many ESCs treat the first move into reverse as "brake", and
//      only reverse if you go neutral -> reverse a second time. reactive_node's
//      stuck-recovery drives at -0.5 m/s and expects the car to actually move
//      backward, so test this specifically.
//   4. OPEN LOOP: a pulse sets motor POWER, not speed. "1.5 m/s" here is only
//      an estimate (THROTTLE_US_PER_MPS in config.h, measured by timing the
//      car over a known distance). Battery voltage, floor and slope change the
//      real speed. Real speed control needs speed_sensor.cpp plus a small PID
//      loop: error = target - measured; pulse += kp*error + ...
//
// THE MATH (same shape as steering)
//   us = THROTTLE_NEUTRAL_US + (mmps / 1000.0) * THROTTLE_US_PER_MPS
//   us = constrain(us, THROTTLE_MIN_US, THROTTLE_MAX_US)   <-- safety clamp;
//   keep it tight (1450..1550) until you've watched it on a stand.
//
// WHAT TO IMPLEMENT (wrap the hardware parts in #if ENABLE_THROTTLE ... #endif
//   so the disabled build contains no code that can drive the pin)
//   throttleBegin():     attach + write neutral.
//   throttleSetMmps():   the math above.
//   throttleNeutral():   write THROTTLE_NEUTRAL_US.
//
// IF THIS BECOMES YOUR OWN ESC BOARD
//   This file is the one that gets rewritten: instead of one pulse to someone
//   else's ESC, you'd run the motor driver yourself (commutation, current
//   limit, closed-loop speed). Keep these three function names as the
//   boundary and main.cpp won't need to change.
// ---------------------------------------------------------------------------

#include "throttle.h"
#include "config.h"

void throttleBegin() {
  // TODO (later)
}

void throttleSetMmps(int16_t mmps) {
  // TODO (later)
  (void)mmps;
}

void throttleNeutral() {
  // TODO (later)
}

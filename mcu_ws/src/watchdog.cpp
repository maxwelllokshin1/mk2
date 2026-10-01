// watchdog.cpp — "if the Pi goes quiet, stop." The most important safety
// feature in the whole system, and it's about ten lines.
//
// ---------------------------------------------------------------------------
// WHY IT'S NEEDED
//   Without it, the board just keeps doing the LAST command it received. So
//   if the Pi crashes, the lidar drops out and reactive_node stops publishing
//   (it only publishes when a scan arrives!), the USB cable wiggles loose, or
//   someone kills the ROS process, the car keeps steering and driving at
//   whatever it was doing, straight into a wall. Nothing on the Pi side can
//   fix that: the software that would stop the car is the thing that died.
//   Only the board can notice "I've stopped hearing commands".
//
// HOW IT WORKS
//   The Pi sends a command frame every ~20 ms. Remember WHEN the last valid
//   one arrived (millis() = milliseconds since the board powered on). If more
//   than WATCHDOG_MS (config.h) have passed, the watchdog is "tripped" and
//   main.cpp forces steering to center and throttle to neutral.
//   It clears itself the moment a new valid command arrives.
//
// THREE DETAILS TO GET RIGHT
//   1. START TRIPPED. Right after boot no command has arrived, so a naive
//      "now - last > timeout" could read as fine. Keep a `haveCommand` flag,
//      false until the first watchdogFeed(), and report tripped while false.
//      The car must not act on anything until the Pi has actually spoken.
//   2. millis() WRAPS around to 0 after ~49 days. Compute the elapsed time with
//      UNSIGNED subtraction, which is correct across the wrap:
//          (uint32_t)(millis() - lastCommandMs) > WATCHDOG_MS
//      Don't write  millis() > last + timeout  — that breaks at the wrap.
//   3. Only VALID frames feed it. protocolPoll() returns true only after the
//      CRC passed, so noise can't keep a dead Pi "alive".
//
// NOT THE SAME THING AS THE AVR HARDWARE WATCHDOG (avr/wdt.h)
//   That one resets the chip if the *firmware* hangs. Useful later, but a
//   different problem. This file only watches the *Pi*.
//
// WHAT TO IMPLEMENT
//   file-level statics: uint32_t lastCommandMs; bool haveCommand = false;
//   watchdogFeed():    record millis(), set haveCommand.
//   watchdogTripped(): !haveCommand || elapsed > WATCHDOG_MS.
// HOW TO TEST
//   Run serial_steer_test and command an angle. That tool keeps re-sending, so
//   the wheels hold it. Quit it (Ctrl+C): the wheels must return to center
//   within ~200 ms. Then unplug the USB cable mid-command and check again.
// ---------------------------------------------------------------------------

#include "watchdog.h"
#include "config.h"

void watchdogFeed() {
  // TODO
}

bool watchdogTripped() {
  // TODO: return true until the first feed, then apply the timeout.
  return true;
}

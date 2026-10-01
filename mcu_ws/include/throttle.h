// throttle.h — public face of throttle.cpp. Details and how-to are there.
#pragma once
#include <Arduino.h>

void throttleBegin();                  // arm the ESC (no-op unless ENABLE_THROTTLE)
void throttleSetMmps(int16_t mmps);    // mm/s, + forward, - reverse; clamped inside
void throttleNeutral();                // motor stopped (used by the failsafe)

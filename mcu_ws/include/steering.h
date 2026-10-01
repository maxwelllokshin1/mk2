// steering.h — public face of steering.cpp. Details and how-to are there.
#pragma once
#include <Arduino.h>

void steeringBegin();                  // attach the servo and drive it to center
void steeringSetCdeg(int16_t cdeg);    // centi-degrees, + = left; clamped to safe limits inside
void steeringCenter();                 // wheels straight (used by the failsafe)

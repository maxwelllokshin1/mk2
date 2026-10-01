// speed_sensor.h — public face of speed_sensor.cpp. Details and how-to are there.
#pragma once
#include <Arduino.h>

void    speedSensorBegin();      // set up the pin and its interrupt
int16_t speedSensorMmps();       // measured speed in mm/s, SIGNED (+ forward, - reverse)

// watchdog.h — public face of watchdog.cpp. Details and how-to are there.
#pragma once
#include <Arduino.h>

void watchdogFeed();       // call every time a valid command arrives
bool watchdogTripped();    // true = too long since the last valid command (or none yet)

#!/usr/bin/env bash
# Keeps trying to connect every paired Joy-Con. Press any button on a Joy-Con and it connects within a few seconds.
# Runs forever as the joycon-autoconnect service (see joycon-autoconnect.service).
while true; do
  for mac in $(bluetoothctl devices Paired 2>/dev/null | awk '/Joy-Con/ {print $2}'); do
    if ! bluetoothctl info "$mac" | grep -q 'Connected: yes'; then
      timeout 6 bluetoothctl connect "$mac" >/dev/null 2>&1 || true
    fi
  done
  sleep 2
done

# mcu_ws — car firmware (Arduino now, your own ESC board later)

## What this firmware is for

The Pi runs the driving logic (`reactive_node`). It works in physical units:
"steer 12 degrees left, drive 1.5 m/s". A servo and an ESC don't understand
that. They understand **pulses**: a 1000–2000 µs high pulse repeated 50 times
a second. The firmware is the translator, and the last line of defense if
the Pi misbehaves:

```
Pi (bridge_node) --serial bytes--> [ THIS FIRMWARE ] --pulses--> servo / ESC
                 <--serial bytes--                  <--pulses--  wheel-speed sensor
```

It has exactly four jobs:
1. **Listen** for command frames from the Pi          -> `protocol`
2. **Turn commands into pulses**                      -> `steering`, `throttle`
3. **Stop safely if the Pi goes quiet**               -> `watchdog`
4. **Report back** what the car is actually doing     -> `speed_sensor` + `protocol`

## File map (PlatformIO layout: headers in `include/`, code in `src/`)

| File | Job | Needed for steering-only test? |
|---|---|---|
| `platformio.ini` | Board, framework, how to build/upload | yes |
| `include/config.h` | Every pin, timing and calibration number in ONE place | yes |
| `src/main.cpp` | `setup()` / `loop()`: calls the modules in the right order | yes |
| `include/protocol.h`, `src/protocol.cpp` | Serial framing + checksum, both directions | yes |
| `include/steering.h`, `src/steering.cpp` | Angle -> servo pulse | yes |
| `include/watchdog.h`, `src/watchdog.cpp` | "Pi went quiet" detector | yes |
| `include/throttle.h`, `src/throttle.cpp` | Speed -> ESC pulse (compiled out at first) | no |
| `include/speed_sensor.h`, `src/speed_sensor.cpp` | Wheel speed from hall/encoder | no |

`libs_external/` is for third-party libraries you'd copy in by hand (unused
for now). PlatformIO compiles every `.cpp` in `src/` automatically.

## Suggested order to write it

Each step ends with something you can observe, so you're never debugging
five things at once.

1. **`config.h`** — decide pins. Wire the servo signal to that pin, servo
   power from a separate 5–6 V supply (BEC), and tie all grounds together.
2. **`steering.cpp`** — call `steeringBegin()` from `setup()`. Power up: the
   wheels should sit at center. Hard-code `steeringSetCdeg(500)` in `setup()`
   and confirm the wheels move; find the right sign and limits (calibration
   steps are in `steering.cpp`). Then delete the hard-coded line.
3. **`protocol.cpp`** — the hardest part. Verify against the **test vectors**
   at the top of that file before you ever connect the Pi.
4. **`watchdog.cpp` + `main.cpp`** — wire everything together.
5. **Test with `python3 -m mcu_bridge.serial_steer_test`** (the Pi-side tool).
   Type angles; stop typing and the wheels must re-center after 200 ms.
6. Later: `speed_sensor`, then `throttle` with `ENABLE_THROTTLE 1`.

## Build / upload / debug

```bash
cd ~/mcu_workspaces/racer_mcu           # inside the container
pio run                                 # compile only (works on the empty skeleton)
pio run -t upload --upload-port /dev/ttyACM0
```

**You cannot `Serial.print()` debug text.** The USB serial port is the
protocol channel: extra text would be parsed as garbage frames and corrupt
the stream. Debug with the on-board LED (blink patterns), or an oscilloscope/
logic analyzer on the servo pin. On a board with a second UART (Mega:
`Serial1`) you can print there.

## When you replace the Arduino with your own ESC board

Only the *hardware-touching* files change: `steering`, `throttle`,
`speed_sensor`, and the pins in `config.h`. `protocol` and `watchdog` and the
shape of `main.cpp` carry over unchanged. That's why they're separate.

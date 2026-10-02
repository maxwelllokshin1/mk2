# pi_drive

Joy-Con driving + a Raspberry Pi GPIO servo/ESC driver. No VESC, no MCU, no Ackermann messages
on the motor path.

```
Joy-Cons --BT--> /dev/input --> joycon_teleop --/drive_cmd [steer, throttle]--> gpio_driver --> servo + ESC pins
                                     ^                                              |
                         /drive (Ackermann) from reactive_node <-- /scan <-- urg_node2   +--> /ego_racecar/odom (dead reckoned)
```

| Control | Action |
|---|---|
| Left Joy-Con stick | steering |
| A (hold) | forward |
| B (hold) | reverse |
| R (hold) | autonomy: `follow_the_gap` steers and drives. Release = RC at once |

`/drive_cmd` is a `Float32MultiArray [steering, throttle]`, both -1..1 (+ = left / forward).
`/drive_mode` (String) says `RC`, `AUTO`, `AUTO_WAITING` (R held but no `/drive` yet) or `NO_CONTROLLER`.

## Safety built in
- `gpio_driver` is separate from teleop: no `/drive_cmd` for 0.5 s -> steering centered, throttle neutral.
- ESC held at neutral for 3 s after start; throttle ramps up but drops instantly; forward<->reverse pauses at neutral.
- Pulses stop if the driver process dies. On shutdown it centers first.
- A+B together = stop. Releasing every button = stop.
- Pins default to `-1`: **DRY RUN**, nothing is driven until you set them.

## First-time setup
1. **Pair the Joy-Cons on the Pi (host, not the container)**
   ```bash
   bluetoothctl
   > power on
   > agent on
   > scan on          # hold the small sync button on the rail until the LEDs sweep
   > pair <MAC>       # repeat for left and right
   > trust <MAC>
   > connect <MAC>
   ```
   Check `ls /dev/input/by-id` or `cat /proc/bus/input/devices | grep -i joy-con` — the kernel's
   `hid-nintendo` driver must be present (`lsmod | grep nintendo`, Pi OS Bookworm has it).
2. **Group ids** for the compose file: `getent group gpio dialout input`, then set `GPIO_GID`, `DIALOUT_GID`, `INPUT_GID`.
3. **Check the button names** (they depend on kernel version and how you hold the Joy-Con):
   `docker exec -it racer /pi_entrypoint.sh ros2 run pi_drive joycon_probe`, press A, B, R, move the
   stick, and copy the names it prints into `config/pi_drive.yaml` (`a_button`, `b_button`,
   `autonomy_button`, `steer_axis`, `steer_side`). Defaults are a best guess.
4. **Pins**: set `steer_gpio` and `throttle_gpio` (BCM numbers, not header pin numbers) in `config/pi_drive.yaml`.
   The file is bind-mounted, so just `docker compose -f docker-compose.pi.yml restart`.
5. **Tune with the wheels off the ground**: steering direction (`steer_sign`), end stops
   (`steer_min_us/max_us`), ESC neutral and arming, then `rc_forward` / `rc_reverse`.

## Wiring
Servo and ESC signal wires to the GPIO pins, **grounds shared with the Pi**, servo powered from the
ESC's BEC or a battery — never from the Pi's 5 V pin. 3.3 V logic is fine for almost all servos/ESCs.

## Notes / limits
- Pulses come from `lgpio` software timing: a little jitter when the CPU is busy. Hardware PWM
  (GPIO 12/13/18/19) would be steadier; the output is a small class in `pwm.py`, easy to swap.
- Odometry is dead reckoned from the commands (not measured). `follow_the_gap` only needs it for
  stuck detection; tune `odom_speed_at_full_throttle`.
- Autonomy throttle is a fixed low value (`auto_throttle`), not the 15 m/s `reactive_node` asks for.
- Some car ESCs need a brake-then-neutral sequence before reverse; the driver only guarantees a neutral pause.
- Run the unit tests: `python3 -m pytest test` (no hardware needed).

# mcu_bridge

Connects `follow_the_gap` to real hardware: `/drive` in, `/odom` out.
**This package is a skeleton: the Python files are stubs with TODOs.** The
firmware side, which is the part with the real explanation, is in
[mcu_ws/README.md](../../../../mcu_ws/README.md).

```
reactive_node --/drive--> bridge_node --> backend --> steering servo / ESC
                              ^                |
                              +---- /odom <----+ (telemetry, serial backend only)
```

## Files

| File | What goes in it |
|---|---|
| [protocol.py](mcu_bridge/protocol.py) | CRC-8, frame encode/decode, `FrameParser`. Twin of the firmware's `protocol.cpp`. Has test vectors. |
| [backends/base.py](mcu_bridge/backends/base.py) | The 4-method interface the node talks to (done). |
| [backends/serial_backend.py](mcu_bridge/backends/serial_backend.py) | Arduino / custom ESC over serial. |
| [backends/gpio_backend.py](mcu_bridge/backends/gpio_backend.py) | Pi-only: servo pulses straight from a GPIO pin. |
| [backends/dummy_backend.py](mcu_bridge/backends/dummy_backend.py) | Logs only; runs with no hardware. Write this first: it lets you test all the ROS wiring on a laptop. |
| [bridge_node.py](mcu_bridge/bridge_node.py) | The ROS node: clamp `/drive`, call the backend, publish `/odom`. |
| [steering_sweep.py](mcu_bridge/steering_sweep.py) | Test node: sweeps steering with speed 0. |
| [serial_steer_test.py](mcu_bridge/serial_steer_test.py) | No-ROS terminal tool for testing the firmware. |
| [launch/steering_test.launch.py](launch/steering_test.launch.py) | Starts bridge + steering_sweep. Skeleton; the docstring explains launch files. |
| [launch/real_gap.launch.py](launch/real_gap.launch.py) | Starts lidar + bridge + reactive_node. Skeleton; explains the lifecycle lidar and the odom remap. |
| `config/bridge_params.yaml`, `setup.py`, `package.xml`, `setup.cfg` | Filled in. Edit calibration values in the yaml. |

## Three ways to run it (one parameter: `backend:=`)

| `backend:=` | Hardware | Notes |
|---|---|---|
| `serial` | Arduino now, your own ESC board later | Firmware in `mcu_ws/`. Porting to a custom board = rewriting the hardware-touching firmware files only. |
| `gpio` | Pi only, no MCU | Software-timed pulses, can jitter; no watchdog below the node. Good for testing a servo, not for driving. |
| `dummy` | none | Logs what it would send. |

## Order to build it

1. **No hardware needed:** `dummy_backend.py`, `bridge_node.py`,
   `steering_sweep.py`, `steering_test.launch.py`. Run it with `backend:=dummy`
   and confirm the log shows the steering stepping. That proves the ROS wiring.
2. `protocol.py` (check the test vectors) — pairs with firmware `protocol.cpp`.
3. `serial_backend.py` + `serial_steer_test.py`, then test against the firmware.
4. `ros2 launch mcu_bridge steering_test.launch.py backend:=serial`.
5. `real_gap.launch.py` (Hokuyo + bridge + `reactive_node`, `steering_only` on
   by default). Untested.

Only after steering works everywhere: wire the ESC, set `ENABLE_THROTTLE 1` in
the firmware, and pass `steering_only:=false` with the wheels off the ground.

## Known gaps

- `odom_mode: commanded` fakes odom from the commanded speed, so
  `reactive_node`'s stuck detection never fires until a wheel sensor exists
  (`speed_sensor.cpp`, then `odom_mode: measured`).
- Serial reconnect isn't handled.
- The ESC needs reverse handling (recovery drives at -0.5 m/s).

#!/usr/bin/env python3
"""
joycon_probe: no ROS. Prints every Joy-Con event with its evdev name so you can
fill in steer_axis / a_button / b_button / autonomy_button in pi_drive.yaml.

    ros2 run pi_drive joycon_probe

Press A, B, R and move the stick. Note the NAME printed next to each (e.g.
BTN_EAST, ABS_X). Ctrl+C to quit.
"""
import select
import sys

try:
    import evdev
    from evdev import ecodes
except ImportError:
    sys.exit('python3-evdev is not installed')


def main():
    devs = {}
    for path in evdev.list_devices():
        try:
            d = evdev.InputDevice(path)
        except OSError as exc:
            print(f'cannot open {path}: {exc}  (check the input group / permissions)')
            continue
        print(f'{path}: {d.name}')
        if 'joy-con' in d.name.lower() and 'imu' not in d.name.lower():
            devs[d.fd] = d
    if not devs:
        sys.exit('no Joy-Con found. Pair/connect them first (see README).')
    print('\nlistening, press buttons / move sticks...\n')
    try:
        while True:
            ready, _, _ = select.select(list(devs), [], [])
            for fd in ready:
                d = devs[fd]
                for ev in d.read():
                    if ev.type == ecodes.EV_KEY:
                        names = ecodes.bytype[ecodes.EV_KEY].get(ev.code, ev.code)
                        names = names[0] if isinstance(names, list) else names
                        print(f'{d.name:35s} KEY {names:12s} {"down" if ev.value else "up"}')
                    elif ev.type == ecodes.EV_ABS:
                        name = ecodes.bytype[ecodes.EV_ABS].get(ev.code, ev.code)
                        info = d.absinfo(ev.code)
                        print(f'{d.name:35s} ABS {name:12s} {ev.value:7d}  (range {info.min}..{info.max})')
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
oled_animate.py: play an animated GIF, or a folder of frames, on the 128x64 I2C OLED.
Run on the Pi itself (same setup as oled_test.py).

    ~/oled/bin/python scripts/pi/oled_animate.py anim.gif
    ~/oled/bin/python scripts/pi/oled_animate.py frames/ --fps 12     # frame_001.png, frame_002.png, ... in name order
    ~/oled/bin/python scripts/pi/oled_animate.py anim.gif --once      # play once instead of looping
    ~/oled/bin/python scripts/pi/oled_animate.py                      # built-in bouncing-ball demo

Speed: the display is only as fast as the I2C bus. At the default 100 kHz expect ~10 fps;
at 400 kHz ~25 fps. To raise it, in /boot/firmware/config.txt change the I2C line to
    dtparam=i2c_arm=on,i2c_arm_baudrate=400000
and reboot. (--fps can't beat the bus speed.)
"""
import argparse
import glob
import os
import time

from PIL import Image, ImageDraw, ImageOps, ImageSequence
from luma.core.interface.serial import i2c
from luma.oled.device import sh1106, ssd1306

W, H = 128, 64


def to_frame(img):
    """Any image -> 1-bit 128x64, aspect ratio kept, centered."""
    img = ImageOps.contain(img.convert('L'), (W, H))
    canvas = Image.new('L', (W, H), 0)
    canvas.paste(img, ((W - img.width) // 2, (H - img.height) // 2))
    return canvas.convert('1')


def load_frames(path):
    """Returns (frames, per-frame delays in seconds or None)."""
    if os.path.isdir(path):
        files = sorted(f for ext in ('png', 'jpg', 'jpeg', 'bmp') for f in glob.glob(os.path.join(path, f'*.{ext}')))
        if not files:
            raise SystemExit(f'no png/jpg/bmp frames in {path}')
        return [to_frame(Image.open(f)) for f in files], None
    gif = Image.open(path)
    frames, delays = [], []
    for fr in ImageSequence.Iterator(gif):
        frames.append(to_frame(fr.copy()))
        delays.append(fr.info.get('duration', 100) / 1000.0)
    return frames, delays


def demo_frames(n=40):
    frames = []
    for i in range(n):
        img = Image.new('1', (W, H), 0)
        d = ImageDraw.Draw(img)
        d.rectangle((0, 0, W - 1, H - 1), outline=1)
        t = abs((i / (n - 1)) * 2 - 1)                    # 1 -> 0 -> 1
        x = int(8 + (1 - t) * (W - 24))
        y = int(8 + abs(((i * 3) % 40) / 20 - 1) * (H - 24))
        d.ellipse((x, y, x + 8, y + 8), fill=1)
        frames.append(img)
    return frames, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('source', nargs='?', help='GIF file or folder of frames (default: demo)')
    ap.add_argument('--fps', type=float, default=None, help='frames per second (default: GIF timing, or 15 for folders / demo)')
    ap.add_argument('--once', action='store_true')
    ap.add_argument('--addr', type=lambda s: int(s, 0), default=0x3C)
    ap.add_argument('--rotate', type=int, default=0, choices=[0, 1, 2, 3])
    ap.add_argument('--sh1106', action='store_true')
    args = ap.parse_args()

    device = (sh1106 if args.sh1106 else ssd1306)(i2c(port=1, address=args.addr), width=W, height=H, rotate=args.rotate)
    frames, delays = load_frames(args.source) if args.source else demo_frames()   # all converted ONCE, up front
    default_delay = 1.0 / (args.fps or 15.0)
    print(f'{len(frames)} frames. Ctrl+C to stop.')

    try:
        while True:
            for i, fr in enumerate(frames):
                start = time.monotonic()
                device.display(fr)
                want = delays[i] if delays and not args.fps else default_delay
                spare = want - (time.monotonic() - start)
                if spare > 0:
                    time.sleep(spare)
            if args.once:
                break
    except KeyboardInterrupt:
        pass
    finally:
        device.clear()


if __name__ == '__main__':
    main()

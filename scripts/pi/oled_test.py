#!/usr/bin/env python3
"""
oled_test.py: show a test pattern or an image on a 0.96" 128x64 I2C OLED (SSD1306).
Run this on the Pi itself (not inside the racer container).

Wiring (Pi header pins):  VDD -> 1 (3.3V)   GND -> 6   SCK/SCL -> 5   SDA -> 3

One-time setup:
    sudo apt install -y python3-venv python3-pil i2c-tools
    sudo raspi-config nonint do_i2c 0          # enable I2C, then reboot
    sudo reboot
    i2cdetect -y 1                             # should show 3c (sometimes 3d)
    python3 -m venv --system-site-packages ~/oled && ~/oled/bin/pip install luma.oled

Usage:
    ~/oled/bin/python scripts/pi/oled_test.py                  # test pattern
    ~/oled/bin/python scripts/pi/oled_test.py photo.png        # your image, scaled to 128x64
    ~/oled/bin/python scripts/pi/oled_test.py --addr 0x3D      # if i2cdetect shows 3d
    ~/oled/bin/python scripts/pi/oled_test.py --rotate 2       # upside down (0-3, 90 deg steps)
"""
import argparse
import time

from PIL import Image, ImageDraw, ImageOps
from luma.core.interface.serial import i2c
from luma.oled.device import sh1106, ssd1306


def test_pattern(w, h):
    img = Image.new('1', (w, h), 0)
    d = ImageDraw.Draw(img)
    d.rectangle((0, 0, w - 1, h - 1), outline=1)            # border: shows if edges are cut off
    d.line((0, 0, w - 1, h - 1), fill=1)                    # diagonals: shows if it is mirrored
    d.line((0, h - 1, w - 1, 0), fill=1)
    d.text((8, 4), 'OLED OK', fill=1)
    d.text((8, h - 14), f'{w}x{h}', fill=1)
    return img


def load_image(path, w, h):
    img = Image.open(path).convert('L')
    img = ImageOps.contain(img, (w, h))                      # keep aspect ratio
    canvas = Image.new('L', (w, h), 0)
    canvas.paste(img, ((w - img.width) // 2, (h - img.height) // 2))
    return canvas.convert('1')                               # 1-bit with dithering


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('image', nargs='?')
    ap.add_argument('--addr', type=lambda s: int(s, 0), default=0x3C)
    ap.add_argument('--bus', type=int, default=1)
    ap.add_argument('--rotate', type=int, default=0, choices=[0, 1, 2, 3])
    ap.add_argument('--sh1106', action='store_true', help='use if the picture is shifted 2 px / has a noisy edge column')
    args = ap.parse_args()

    serial = i2c(port=args.bus, address=args.addr)
    device = (sh1106 if args.sh1106 else ssd1306)(serial, width=128, height=64, rotate=args.rotate)
    img = load_image(args.image, device.width, device.height) if args.image \
        else test_pattern(device.width, device.height)
    device.display(img)
    print('displayed. Ctrl+C to clear and exit.')
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        device.clear()


if __name__ == '__main__':
    main()

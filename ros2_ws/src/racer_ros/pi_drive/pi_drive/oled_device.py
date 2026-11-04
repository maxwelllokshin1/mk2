"""
oled_device.py: the two ways the node can show a frame.

LumaDisplay  the real SSD1306 / SH1106 over I2C (needs /dev/i2c-1 and luma.oled)
MultiDisplay  several LumaDisplays, one picture per screen (two eyes), reconnects a screen that was late
NullDisplay  no hardware: keeps the last frame so it can be saved as a preview PNG
"""


class NullDisplay:
    live = False

    def __init__(self):
        self.last = None

    def show(self, img):
        self.last = img[0] if isinstance(img, (list, tuple)) else img

    def close(self):
        pass


class LumaDisplay:
    live = True

    def __init__(self, bus=1, address=0x3C, driver='ssd1306', rotate=0):
        from luma.core.interface.serial import i2c
        from luma.oled.device import sh1106, ssd1306
        serial = i2c(port=bus, address=address)
        cls = sh1106 if driver == 'sh1106' else ssd1306
        self.device = cls(serial, width=128, height=64, rotate=rotate)
        self.last = None

    def show(self, img):
        self.device.display(img)
        self.last = img

    def close(self):
        try:
            self.device.clear()
        except Exception:
            pass


class MultiDisplay:
    """
    Several screens. show([img1, img2]) gives screen 1 img1 and screen 2 img2 (a short list repeats its last
    picture). A screen that fails to open is skipped and retried by retry(), so a late-connected or
    loose second screen comes alive without restarting. A write failure raises, so the node reopens all.
    """
    live = True

    def __init__(self, specs):
        self.specs, self.slots, self.errors = list(specs), [None] * len(specs), {}
        self.retry()
        if not self.devices:
            raise RuntimeError('; '.join(self.errors.values()))
        self.last = None

    @property
    def devices(self):
        return [d for d in self.slots if d is not None]

    @property
    def missing(self):
        return [self.specs[i] for i, d in enumerate(self.slots) if d is None]

    def retry(self):
        """Try to open every screen that is not up yet. Returns how many came up now."""
        came_up = 0
        for i, spec in enumerate(self.specs):
            if self.slots[i] is None:
                try:
                    self.slots[i] = LumaDisplay(*spec)
                    self.errors.pop(i, None)
                    came_up += 1
                except Exception as exc:
                    self.errors[i] = f'bus {spec[0]} addr {spec[1]:#04x}: {exc}'
        return came_up

    def show(self, images):
        if not isinstance(images, (list, tuple)):
            images = [images]
        for i, dev in enumerate(self.slots):
            if dev is not None:
                dev.show(images[min(i, len(images) - 1)])
        self.last = images[0]

    def close(self):
        for dev in self.devices:
            dev.close()

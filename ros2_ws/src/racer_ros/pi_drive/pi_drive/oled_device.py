"""
oled_device.py: the two ways the node can show a frame.

LumaDisplay  the real SSD1306 / SH1106 over I2C (needs /dev/i2c-1 and luma.oled)
NullDisplay  no hardware: keeps the last frame so it can be saved as a preview PNG
"""


class NullDisplay:
    live = False

    def __init__(self):
        self.last = None

    def show(self, img):
        self.last = img

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

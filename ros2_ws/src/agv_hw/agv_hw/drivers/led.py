"""빨강 GPIO16 · 초록 GPIO26 LED (wiring.md)."""
from .base import LedDriver
from .gpio import open_header_chip


class LgpioLedDriver(LedDriver):
    def __init__(self, red_pin: int = 16, green_pin: int = 26, chip: int | None = None):
        self.lgpio, self.h = open_header_chip(chip)
        self.red, self.green = red_pin, green_pin
        for pin in (red_pin, green_pin):
            self.lgpio.gpio_claim_output(self.h, pin, 0)

    def set(self, red: bool, green: bool) -> None:
        self.lgpio.gpio_write(self.h, self.red, int(red))
        self.lgpio.gpio_write(self.h, self.green, int(green))

    def close(self) -> None:
        self.set(False, False)
        for pin in (self.red, self.green):
            self.lgpio.gpio_free(self.h, pin)
        self.lgpio.gpiochip_close(self.h)

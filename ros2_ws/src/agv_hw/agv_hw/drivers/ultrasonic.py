"""HC-SR04 — Trig 10µs → Echo 펄스 폭 × 343/2. Echo는 1k+2k 분압 후 GPIO6 (wiring.md).

Echo 시간은 lgpio 경계 콜백의 타임스탬프(ns)로 재서 Python 반복문 지연의 영향을 줄인다.
"""
import threading
import time

from .base import UltrasonicDriver

SPEED_OF_SOUND = 343.0          # m/s (20℃)


def echo_to_distance(width_ns: int) -> float:
    return width_ns * 1e-9 * SPEED_OF_SOUND / 2


class LgpioUltrasonicDriver(UltrasonicDriver):
    def __init__(self, trig_pin: int = 5, echo_pin: int = 6, timeout_s: float = 0.03, chip: int | None = None):
        from .gpio import open_header_chip
        self.lgpio, self.h = open_header_chip(chip)
        self.trig, self.echo, self.timeout_s = trig_pin, echo_pin, timeout_s
        self.lgpio.gpio_claim_output(self.h, trig_pin, 0)
        self.lgpio.gpio_claim_alert(self.h, echo_pin, self.lgpio.BOTH_EDGES)
        self._rise = None
        self._width = None
        self._done = threading.Event()
        self._cb = self.lgpio.callback(self.h, echo_pin, self.lgpio.BOTH_EDGES, self._on_edge)

    def _on_edge(self, chip, gpio, level, tick):
        if level == 1:
            self._rise = tick
        elif level == 0 and self._rise is not None:
            self._width = tick - self._rise
            self._done.set()

    def measure(self) -> float | None:
        self._rise, self._width = None, None
        self._done.clear()
        self.lgpio.gpio_write(self.h, self.trig, 1)
        time.sleep(0.00001)
        self.lgpio.gpio_write(self.h, self.trig, 0)
        if not self._done.wait(self.timeout_s):
            return None
        return echo_to_distance(self._width)

    def close(self) -> None:
        self._cb.cancel()
        for pin in (self.trig, self.echo):
            try:
                self.lgpio.gpio_free(self.h, pin)
            except self.lgpio.error:
                pass
        self.lgpio.gpiochip_close(self.h)

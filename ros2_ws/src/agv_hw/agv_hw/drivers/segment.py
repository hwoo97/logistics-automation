"""7세그먼트 1자리 — a~g를 GPIO 7개로 직접 구동, 세그먼트마다 330Ω (wiring.md, 2026-11-02)."""
from .base import SegmentDriver
from .gpio import open_header_chip

DEFAULT_PINS = (4, 27, 12, 13, 7, 10, 11)          # a, b, c, d, e, f, g — pinmap.md (e: GPIO9 → 7, 2026-11-09)


class LgpioSegmentDriver(SegmentDriver):
    def __init__(self, pins=DEFAULT_PINS, common_anode: bool = False, chip: int | None = None):
        self.pins = tuple(pins)
        self.common_anode = common_anode            # 공통 애노드면 LOW = 켬
        self.lgpio, self.h = open_header_chip(chip)
        for pin in self.pins:
            self.lgpio.gpio_claim_output(self.h, pin, int(common_anode))

    def set(self, bits: tuple) -> None:
        for pin, on in zip(self.pins, bits):
            self.lgpio.gpio_write(self.h, pin, int(bool(on) != self.common_anode))

    def close(self) -> None:
        try:
            self.set((0,) * 7)
        except self.lgpio.error:
            pass
        for pin in self.pins:
            try:
                self.lgpio.gpio_free(self.h, pin)
            except self.lgpio.error:
                pass
        self.lgpio.gpiochip_close(self.h)

"""PCA9685(I2C) 서보 — 50Hz, 각도 → 펄스 폭(0.5~2.5ms). 03_setup/gpio_i2c/pca9685_check.py에서 검증한 순서."""
import time

from .base import ServoDriver

MODE1, PRESCALE, LED0_ON_L = 0x00, 0xFE, 0x06
OSC_HZ = 25_000_000


def prescale_for(freq_hz: float) -> int:
    return round(OSC_HZ / (4096 * freq_hz)) - 1


def angle_to_counts(deg: float, freq_hz: float = 50, pulse_min_us: float = 500, pulse_max_us: float = 2500) -> int:
    deg = max(0.0, min(180.0, deg))
    pulse_us = pulse_min_us + (pulse_max_us - pulse_min_us) * deg / 180.0
    return round(pulse_us / (1_000_000 / freq_hz) * 4096)


class Pca9685ServoDriver(ServoDriver):
    def __init__(self, bus: int = 1, address: int = 0x40, oe_pin: int | None = 17, freq_hz: float = 50,
                 pulse_min_us: float = 500, pulse_max_us: float = 2500, chip: int | None = None):
        from smbus2 import SMBus
        self.bus = SMBus(bus)
        self.address = address
        self.freq_hz, self.pulse = freq_hz, (pulse_min_us, pulse_max_us)
        self._oe = None
        if oe_pin is not None:                  # OE(LOW = 출력 활성) 추정 핀 — 연결 안 돼 있어도 무해
            from .gpio import open_header_chip
            lgpio, h = open_header_chip(chip)
            lgpio.gpio_claim_output(h, oe_pin, 0)
            self._oe = (lgpio, h, oe_pin)
        self._set_freq(freq_hz)

    def _w(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value)

    def _set_freq(self, freq_hz):
        old = self.bus.read_byte_data(self.address, MODE1) & 0x7F
        self._w(MODE1, old | 0x10)              # SLEEP 상태에서만 PRESCALE 변경 가능
        self._w(PRESCALE, prescale_for(freq_hz))
        awake = (old & ~0x10) | 0x20            # 깨우기 + 자동 증가
        self._w(MODE1, awake)
        time.sleep(0.001)
        self._w(MODE1, awake | 0x80)            # RESTART

    def _set_counts(self, channel, on, off):
        base = LED0_ON_L + 4 * channel
        for i, v in enumerate((on & 0xFF, on >> 8, off & 0xFF, off >> 8)):
            self._w(base + i, v)

    def set_angle(self, channel: int, deg: float) -> None:
        self._set_counts(channel, 0, angle_to_counts(deg, self.freq_hz, *self.pulse))

    def release(self, channel: int) -> None:
        """출력 끔 (서보 힘 풀림)."""
        self._set_counts(channel, 0, 0x1000)

    def close(self) -> None:
        if self._oe:
            lgpio, h, pin = self._oe
            lgpio.gpio_free(h, pin)
            lgpio.gpiochip_close(h)
        self.bus.close()

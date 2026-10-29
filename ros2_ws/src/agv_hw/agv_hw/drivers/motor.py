"""DRV8835(PHASE/ENABLE) × 2 — 바퀴 4개 개별 제어, lgpio 소프트웨어 PWM."""
from .base import BOARD_WHEELS, MotorDriver
from .gpio import open_header_chip

DEFAULT_PINS = {"L1": (18, 19), "L2": (20, 21), "L3": (22, 23), "L4": (24, 25)}   # (DIR, PWM) — pinmap.md


class LgpioMotorDriver(MotorDriver):
    def __init__(self, wheel_pins: dict | None = None, duty_max: float = 0.75, pwm_hz: int = 1000,
                 invert: dict | None = None, chip: int | None = None):
        self.pins = wheel_pins or DEFAULT_PINS
        self.duty_max = max(0.0, min(1.0, duty_max))
        self.pwm_hz = pwm_hz
        self.invert = invert or {}
        self.lgpio, self.h = open_header_chip(chip)
        for dir_pin, pwm_pin in self.pins.values():
            self.lgpio.gpio_claim_output(self.h, dir_pin, 0)
            self.lgpio.gpio_claim_output(self.h, pwm_pin, 0)

    def set_wheel(self, wheel: str, speed: float) -> None:
        dir_pin, pwm_pin = self.pins[wheel]
        speed = max(-1.0, min(1.0, speed))
        if self.invert.get(wheel):
            speed = -speed
        self.lgpio.gpio_write(self.h, dir_pin, 1 if speed < 0 else 0)
        duty = abs(speed) * self.duty_max * 100.0
        if duty < 0.5:
            # 주파수 0은 lgpio가 'bad PWM micros'로 거부 → 듀티 0으로 PWM 정지 (2026-10-29 Pi에서 확인)
            self.lgpio.tx_pwm(self.h, pwm_pin, self.pwm_hz, 0)
            self.lgpio.gpio_write(self.h, pwm_pin, 0)
        else:
            self.lgpio.tx_pwm(self.h, pwm_pin, self.pwm_hz, duty)

    def stop_all(self) -> None:
        for wheel in self.pins:
            self.set_wheel(wheel, 0.0)

    def close(self) -> None:
        try:
            self.stop_all()
        except self.lgpio.error:
            pass  # 정지에 실패해도 핀은 풀어서 다음 실행이 막히지 않게 한다
        for dir_pin, pwm_pin in self.pins.values():
            for pin in (dir_pin, pwm_pin):
                try:
                    self.lgpio.gpio_free(self.h, pin)
                except self.lgpio.error:
                    pass
        self.lgpio.gpiochip_close(self.h)


__all__ = ["LgpioMotorDriver", "DEFAULT_PINS", "BOARD_WHEELS"]

"""LgpioMotorDriver — 실제 lgpio의 인자 검사를 흉내 낸 가짜 모듈로 시험 (Pi 없이)."""
import pytest

from agv_hw.drivers import motor


class StrictLgpio:
    """lgpio처럼 잘못된 PWM 인자를 거부한다 (주파수 0.1~10000Hz, 듀티 0~100)."""

    class error(Exception):
        pass

    def __init__(self):
        self.pwm = {}       # 핀 → (주파수, 듀티)
        self.level = {}     # 핀 → 0/1
        self.claimed = set()

    def gpio_claim_output(self, h, pin, level):
        self.claimed.add(pin)
        self.level[pin] = level

    def gpio_write(self, h, pin, level):
        self.level[pin] = level

    def tx_pwm(self, h, pin, freq, duty):
        if not 0.1 <= freq <= 10000 or not 0 <= duty <= 100:
            raise self.error("bad PWM micros")
        self.pwm[pin] = (freq, duty)

    def gpio_free(self, h, pin):
        self.claimed.discard(pin)

    def gpiochip_close(self, h):
        pass


@pytest.fixture
def drv(monkeypatch):
    fake = StrictLgpio()
    monkeypatch.setattr(motor, "open_header_chip", lambda chip=None: (fake, 0))
    d = motor.LgpioMotorDriver(duty_max=0.75, pwm_hz=1000, invert={"L2": True})
    d.fake = fake
    return d


def test_stop_does_not_use_zero_frequency(drv):
    drv.set_wheel("L1", 0.0)               # 2026-10-29 Pi: tx_pwm(…, 0, 0) → 'bad PWM micros'로 노드 종료
    dir_pin, pwm_pin = motor.DEFAULT_PINS["L1"]
    assert drv.fake.pwm[pwm_pin] == (1000, 0)
    assert drv.fake.level[pwm_pin] == 0


def test_speed_and_direction(drv):
    drv.set_wheel("L1", 0.8)
    dir_pin, pwm_pin = motor.DEFAULT_PINS["L1"]
    assert drv.fake.level[dir_pin] == 0
    assert drv.fake.pwm[pwm_pin] == (1000, pytest.approx(60.0))   # 0.8 × 0.75
    drv.set_wheel("L1", -1.0)
    assert drv.fake.level[dir_pin] == 1
    assert drv.fake.pwm[pwm_pin][1] == pytest.approx(75.0)


def test_invert(drv):
    drv.set_wheel("L2", 0.5)
    assert drv.fake.level[motor.DEFAULT_PINS["L2"][0]] == 1


def test_stop_all_and_close_free_pins(drv):
    for w in motor.DEFAULT_PINS:
        drv.set_wheel(w, 0.5)
    drv.close()
    assert all(duty == 0 for _, duty in drv.fake.pwm.values())
    assert not drv.fake.claimed

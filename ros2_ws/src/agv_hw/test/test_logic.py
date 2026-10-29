import pytest

from agv_hw.drivers.fake import FakeLedDriver, FakeMotorDriver, FakeServoDriver, FakeUltrasonicDriver
from agv_hw.drivers.servo_pca9685 import angle_to_counts, prescale_for
from agv_hw.drivers.ultrasonic import echo_to_distance
from agv_hw.logic import MedianFilter, Watchdog, led_output, mix, wheel_speeds

WHEEL_MAP = {"L1": "front_left", "L2": "rear_left", "L3": "front_right", "L4": "rear_right"}


def test_median_filter_drops_invalid():
    f = MedianFilter(window=3)
    assert f.push(None) is None
    assert f.push(0.5) == 0.5
    assert f.push(5.0) == 0.5             # 범위 밖 버림
    assert f.push(0.01) == 0.5            # 2cm 미만 버림
    f.push(0.7)
    assert f.push(0.6) == 0.6             # 0.5, 0.7, 0.6 중앙값


@pytest.mark.parametrize("x, z, expect", [
    (0.5, 0.0, (0.5, 0.5)),
    (0.0, 0.5, (-0.5, 0.5)),              # 왼쪽 회전: 왼쪽 뒤로, 오른쪽 앞으로
    (1.0, 1.0, (0.0, 1.0)),               # 2.0 → 1.0으로 비율 유지
    (-0.4, 0.0, (-0.4, -0.4)),
])
def test_mix(x, z, expect):
    assert mix(x, z) == pytest.approx(expect)


def test_wheel_speeds():
    assert wheel_speeds(0.2, 0.8, WHEEL_MAP) == {"L1": 0.2, "L2": 0.2, "L3": 0.8, "L4": 0.8}


@pytest.mark.parametrize("state, blink, out", [
    ("OFF", True, (False, False)), ("RED", False, (True, False)), ("GREEN", True, (False, True)),
    ("ERROR", True, (True, True)), ("ERROR", False, (False, False)), ("??", True, (False, False)),
])
def test_led_output(state, blink, out):
    assert led_output(state, blink) == out


def test_watchdog():
    w = Watchdog(0.5)
    assert w.expired(0.0)
    w.feed(1.0)
    assert not w.expired(1.4) and w.expired(1.6)


def test_servo_math():
    assert prescale_for(50) == 121
    assert (angle_to_counts(0), angle_to_counts(90), angle_to_counts(180)) == (102, 307, 512)
    assert angle_to_counts(-10) == 102 and angle_to_counts(999) == 512


def test_echo_to_distance():
    assert echo_to_distance(1_000_000) == pytest.approx(0.1715)   # 1ms → 17.15cm


def test_fake_drivers():
    m = FakeMotorDriver()
    m.set_wheel("L1", 0.3)
    m.stop_all()
    assert m.speeds["L1"] == 0.0 and m.history[0] == ("set_wheel", "L1", 0.3)
    s = FakeServoDriver()
    s.set_angle(0, 90)
    assert s.angles == {0: 90}
    assert FakeUltrasonicDriver(distance=0.2).measure() == 0.2
    led = FakeLedDriver()
    led.set(True, False)
    led.close()
    assert led.state == (False, False)


def test_s0_wheel_map_forward_and_turn():
    """2026-10-29 S0 확정 배치: L1 앞왼 · L2 앞오 · L3 뒤왼 · L4 뒤오 (반전은 드라이버에서)."""
    s0 = {"L1": "front_left", "L2": "front_right", "L3": "rear_left", "L4": "rear_right"}
    assert wheel_speeds(0.5, 0.5, s0) == {"L1": 0.5, "L2": 0.5, "L3": 0.5, "L4": 0.5}
    assert wheel_speeds(-0.3, 0.3, s0) == {"L1": -0.3, "L2": 0.3, "L3": -0.3, "L4": 0.3}

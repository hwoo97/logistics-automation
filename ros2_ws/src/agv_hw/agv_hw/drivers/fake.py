"""가짜 드라이버 — 하드웨어 없이 노드를 시험 (`use_fake_hw: true`). 호출 내용을 history에 남긴다."""
from .base import MotorDriver, ServoDriver, UltrasonicDriver, LedDriver


class FakeMotorDriver(MotorDriver):
    def __init__(self, *args, **kwargs):
        self.history: list = []
        self.speeds: dict = {}

    def set_wheel(self, wheel, speed):
        self.speeds[wheel] = speed
        self.history.append(("set_wheel", wheel, speed))

    def stop_all(self):
        for w in list(self.speeds) or ["L1", "L2", "L3", "L4"]:
            self.speeds[w] = 0.0
        self.history.append(("stop_all",))


class FakeServoDriver(ServoDriver):
    def __init__(self, *args, **kwargs):
        self.history: list = []
        self.angles: dict = {}

    def set_angle(self, channel, deg):
        self.angles[channel] = deg
        self.history.append(("set_angle", channel, deg))

    def release(self, channel):
        self.history.append(("release", channel))


class FakeUltrasonicDriver(UltrasonicDriver):
    def __init__(self, *args, distance: float | None = 1.0, **kwargs):
        self.distance = distance
        self.history: list = []

    def measure(self):
        self.history.append(("measure",))
        return self.distance


class FakeLedDriver(LedDriver):
    def __init__(self, *args, **kwargs):
        self.history: list = []
        self.state = (False, False)

    def set(self, red, green):
        self.state = (bool(red), bool(green))
        self.history.append(("set", bool(red), bool(green)))

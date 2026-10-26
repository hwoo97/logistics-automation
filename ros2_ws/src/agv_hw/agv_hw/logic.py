"""하드웨어 노드의 순수 로직 (② 계층) — ROS 없이 pytest로 검사한다."""
import statistics
from collections import deque

from agv_protocol.types import LedState


class MedianFilter:
    """이동 중앙값. None·범위 밖 값은 버리고, 유효값이 없으면 None."""

    def __init__(self, window: int = 5, min_value: float = 0.02, max_value: float = 2.0):
        self.values: deque = deque(maxlen=window)
        self.min_value, self.max_value = min_value, max_value

    def push(self, value: float | None) -> float | None:
        if value is not None and self.min_value <= value <= self.max_value:
            self.values.append(value)
        return statistics.median(self.values) if self.values else None


def mix(x: float, z: float, turn_gain: float = 1.0) -> tuple[float, float]:
    """/cmd_vel(x 선속도, z 각속도, 각 -1~1) → (왼쪽, 오른쪽) 속도. z > 0 = 왼쪽으로 회전."""
    left, right = x - z * turn_gain, x + z * turn_gain
    peak = max(1.0, abs(left), abs(right))          # 한쪽이 넘치면 비율 유지하며 줄임
    return left / peak, right / peak


def wheel_speeds(left: float, right: float, wheel_map: dict) -> dict:
    """좌우 속도 → 보드 출력(L1~L4)별 속도. wheel_map: {'L1': 'front_left', ...}"""
    return {board: (left if name.endswith("left") else right) for board, name in wheel_map.items()}


def led_output(state: str, blink_on: bool) -> tuple[bool, bool]:
    """LED 상태 → (빨강, 초록). ERROR는 둘 다 깜빡임 (functional_design F9)."""
    if state == LedState.RED:
        return True, False
    if state == LedState.GREEN:
        return False, True
    if state == LedState.ERROR:
        return blink_on, blink_on
    return False, False


class Watchdog:
    """마지막 입력 후 timeout_s가 지나면 expired."""

    def __init__(self, timeout_s: float = 0.5):
        self.timeout_s = timeout_s
        self.last = None

    def feed(self, now: float) -> None:
        self.last = now

    def expired(self, now: float) -> bool:
        return self.last is None or now - self.last > self.timeout_s

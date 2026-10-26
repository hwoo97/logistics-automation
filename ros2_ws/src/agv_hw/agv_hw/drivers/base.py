"""드라이버 인터페이스 — class_design.md 1.1절."""
from abc import ABC, abstractmethod

WHEELS = ("front_left", "rear_left", "front_right", "rear_right")
BOARD_WHEELS = ("L1", "L2", "L3", "L4")          # 제어보드 모터 출력 이름 (pinmap.md)


class MotorDriver(ABC):
    @abstractmethod
    def set_wheel(self, wheel: str, speed: float) -> None: ...   # wheel: L1~L4, speed: -1.0(후진) ~ 1.0(전진)

    @abstractmethod
    def stop_all(self) -> None: ...

    def close(self) -> None:
        self.stop_all()


class ServoDriver(ABC):
    @abstractmethod
    def set_angle(self, channel: int, deg: float) -> None: ...   # 0 ~ 180°

    def close(self) -> None: ...


class UltrasonicDriver(ABC):
    @abstractmethod
    def measure(self) -> float | None: ...                       # m, 실패 시 None

    def close(self) -> None: ...


class LedDriver(ABC):
    @abstractmethod
    def set(self, red: bool, green: bool) -> None: ...

    def close(self) -> None:
        self.set(False, False)

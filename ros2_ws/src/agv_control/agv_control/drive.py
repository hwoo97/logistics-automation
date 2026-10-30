"""주행 제어 (② 순수 로직) — 라인 상태 → /cmd_vel (functional_design F4·F5, class_design 3.1).

지금 구현: FOLLOW_TO_EVENT(라인 따라가다 이벤트에서 정지), STOP. TURN·STRAIGHT·UTURN·APPROACH는 S2·S7·S8에서 추가.
부호: 오프셋 + = 라인이 오른쪽 → 오른쪽으로 돌아야 함 → /cmd_vel z는 음수 (z > 0 = 왼쪽 회전, agv_hw.logic.mix).
각도 보정: 라인 각도가 90° ± heading_deadband_deg(85~95°)를 벗어나면 벗어난 만큼 회전에 더함.
  각도 < 85 (위쪽이 오른쪽으로 기움) → 오른쪽으로, 각도 > 95 → 왼쪽으로. 회전 = −(PID(오프셋) + kh × 각도 오차/45°)
"""
from dataclasses import dataclass

from agv_protocol.types import DriveCommand, LineEvent

# FOLLOW에서 멈추는 이벤트. 분기점은 색 마커 인식이 판단 (2026-10-30 결정) — 라인 검출은 JUNCTION을 내지 않음
STOP_EVENTS = {LineEvent.STOP_LINE, LineEvent.CORNER_LEFT, LineEvent.CORNER_RIGHT, LineEvent.LINE_END}


@dataclass
class DriveConfig:
    speed: float = 0.6              # 기본 선속도 (-1~1 비율). TT 모터는 낮으면 바닥에서 안 움직임 — S1에서 보정
    min_speed: float = 0.45         # 오프셋이 커서 감속해도 이 아래로는 안 내림 (모터 시동 문턱)
    slow_gain: float = 0.5          # 선속도 = speed × (1 − slow_gain × |오프셋|)
    event_speed: float = 0.45       # 이벤트 판정(확인 전)이 보이기 시작하면 이 속도로 감속
    kp: float = 0.8
    ki: float = 0.0
    kd: float = 0.15
    kh: float = 0.5                 # 각도 보정 비율 (각도 오차 45°일 때 회전 0.5)
    heading_deadband_deg: float = 5.0   # 90°에서 이 안쪽(85~95°)은 각도 보정 안 함
    max_angular: float = 0.6
    line_timeout_s: float = 0.5     # 라인 상태가 이 시간 동안 안 오면 정지 (E_LINE)


@dataclass
class LineInput:
    """line_node의 /line/state와 같은 내용 (agv_vision에 의존하지 않도록 따로 둠)."""
    offset: float
    line_found: bool
    event: LineEvent = LineEvent.NONE
    raw_event: LineEvent = LineEvent.NONE
    angle: float = 90.0             # 라인 각도(도), 90 = 똑바로
    angle_valid: bool = False


@dataclass
class DriveOutput:
    linear: float                   # /cmd_vel x
    angular: float                  # /cmd_vel z
    done: bool = False
    result: str = ""                # 끝난 이유: 이벤트 이름, STOPPED, E_LINE


class PidController:
    def __init__(self, kp: float, ki: float = 0.0, kd: float = 0.0, i_limit: float = 1.0):
        self.kp, self.ki, self.kd, self.i_limit = kp, ki, kd, i_limit
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self.prev = None

    def update(self, error: float, dt: float) -> float:
        dt = max(dt, 1e-3)
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral + error * dt))
        deriv = 0.0 if self.prev is None else (error - self.prev) / dt
        self.prev = error
        return self.kp * error + self.ki * self.integral + self.kd * deriv


def _clip(v: float, lim: float) -> float:
    return max(-lim, min(lim, v))


def heading_error(angle: float, deadband: float) -> float:
    """라인 각도 → 각도 오차(도, 허용 범위만큼 뺀 값). + = 위쪽이 오른쪽으로 기움 → 오른쪽으로 돌아야 함."""
    err = 90.0 - angle
    if abs(err) <= deadband:
        return 0.0
    return err - deadband if err > 0 else err + deadband


class DriveController:
    def __init__(self, cfg: DriveConfig | None = None):
        self.cfg = cfg or DriveConfig()
        self.pid = PidController(self.cfg.kp, self.cfg.ki, self.cfg.kd)
        self.command: DriveCommand | None = None
        self.since_line = 0.0               # 마지막 라인 상태 이후 시간
        self.last = DriveOutput(0.0, 0.0)   # 새 라인 상태가 없는 주기에는 직전 출력 유지
        self._floor = (self.cfg.min_speed, self.cfg.event_speed)   # set_speed가 낮췄다 올릴 때 되돌릴 값

    @property
    def active(self) -> bool:
        return self.command is not None

    def set_speed(self, speed: float) -> None:
        """주행 기본 속도 변경 (노트북 앱, 주행 중에도). 감속 하한·이벤트 감속이 기본 속도보다 크지 않게 맞춘다."""
        self.cfg.speed = speed
        self.cfg.min_speed = min(self._floor[0], speed)
        self.cfg.event_speed = min(self._floor[1], speed)

    def start(self, command: DriveCommand) -> None:
        self.command = DriveCommand(command)
        self.pid.reset()
        self.since_line = 0.0
        self.last = DriveOutput(0.0, 0.0)

    def cancel(self) -> DriveOutput:
        self.command = None
        return DriveOutput(0.0, 0.0, True, "STOPPED")

    def step(self, line: LineInput | None, dt: float) -> DriveOutput:
        """line: 이번 주기에 새로 받은 라인 상태(없으면 None). dt: 주기(초)."""
        if self.command is None:
            return DriveOutput(0.0, 0.0)
        if self.command == DriveCommand.STOP:
            return self.cancel()
        if self.command != DriveCommand.FOLLOW_TO_EVENT:
            name, self.command = self.command.value, None
            return DriveOutput(0.0, 0.0, True, f"E_UNSUPPORTED_{name}")

        self.since_line = 0.0 if line is not None else self.since_line + dt
        if self.since_line > self.cfg.line_timeout_s:
            self.command = None
            return DriveOutput(0.0, 0.0, True, "E_LINE")
        if line is None:                       # 이번 주기엔 새 영상 없음 → 직전 출력 유지
            return self.last
        if line.event in STOP_EVENTS:
            self.command = None
            return DriveOutput(0.0, 0.0, True, line.event.value)

        c = self.cfg
        heading = heading_error(line.angle, c.heading_deadband_deg) / 45.0 if line.angle_valid else 0.0
        angular = _clip(-(self.pid.update(line.offset, dt) + c.kh * heading), c.max_angular)
        linear = max(c.min_speed, c.speed * (1.0 - c.slow_gain * abs(line.offset)))
        if line.raw_event != LineEvent.NONE or not line.line_found:
            linear = min(linear, c.event_speed)          # 이벤트가 보이기 시작하면 감속
        self.last = DriveOutput(round(linear, 3), round(angular, 3))
        return self.last

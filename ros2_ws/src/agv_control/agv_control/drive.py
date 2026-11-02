"""주행 제어 (② 순수 로직) — 라인 상태 → /cmd_vel (functional_design F4·F5, class_design 3.1).

지금 구현: FOLLOW_TO_EVENT(라인 따라가다 이벤트에서 정지), STOP. TURN·STRAIGHT·UTURN·APPROACH는 S2·S7·S8에서 추가.
부호: 오프셋 + = 라인이 오른쪽 → 오른쪽으로 돌아야 함 → /cmd_vel z는 음수 (z > 0 = 왼쪽 회전, agv_hw.logic.mix).
각도 보정: 라인 각도가 90° ± heading_deadband_deg(85~95°)를 벗어나면 벗어난 만큼 회전에 더함.
  각도 < 85 (위쪽이 오른쪽으로 기움) → 오른쪽으로, 각도 > 95 → 왼쪽으로.
회전 = −turn_scale × (PID(오프셋 − center_offset) + kh × 각도 오차/45°)
  center_offset: 카메라가 차체 중심에서 비켜 달려 있을 때 라인이 와야 할 화면 위치 (−0.1 = 가운데에서 10% 왼쪽)
  turn_scale   : 회전 세기 — 노트북 앱에서 주행 중 조정 (흔들리면 낮춤)
마커 (목적지 색이 정해졌을 때만, 2026-11-02): 목적지 색 스티커가 왼쪽만 = 좌회전, 오른쪽만 = 우회전, 양쪽 = 직진.
  보이면 감속 → 화면 아래쪽까지 오면 정지해 여러 프레임으로 좌우 확정 → 전진 → 제자리 회전.
  회전은 각도가 아니라 라인으로 끝낸다 (엔코더 없음): 기존 라인이 벗어난 뒤, 새 라인이 기준 근처에 세로에 가깝게 보이면 끝.
"""
from dataclasses import dataclass

from agv_protocol.types import DriveCommand, LineEvent

# FOLLOW에서 멈추는 이벤트. 분기점은 색 마커 인식이 판단 (2026-10-30 결정) — 라인 검출은 JUNCTION을 내지 않음
STOP_EVENTS = {LineEvent.STOP_LINE, LineEvent.CORNER_LEFT, LineEvent.CORNER_RIGHT, LineEvent.LINE_END}
# 판정이 보이면 먼저 멈추고 확인하는 이벤트 — 달리면서 프레임을 채우면 지나쳐 버림 (2026-11-02 S1)
VERIFY_EVENTS = {LineEvent.STOP_LINE, LineEvent.CORNER_LEFT, LineEvent.CORNER_RIGHT}


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
    turn_scale: float = 1.0         # 회전 세기 (회전량 전체에 곱함, DEV TUNE TURN)
    center_offset: float = 0.0      # 라인 기준 위치 (오프셋 목표, 음수 = 화면 왼쪽, DEV TUNE CENTER)
    heading_deadband_deg: float = 5.0   # 90°에서 이 안쪽(85~95°)은 각도 보정 안 함
    max_angular: float = 0.6
    line_timeout_s: float = 0.5     # 라인 상태가 이 시간 동안 안 오면 정지 (E_LINE)
    recheck_tries: int = 3          # LINE_END 후 정지한 채 라인을 다시 보는 횟수 (0이면 바로 끝)
    recheck_s: float = 0.5          # 한 번 보는 시간
    recheck_frames: int = 3         # 라인이 이만큼 연속 보이면 자동으로 다시 출발
    verify_clear_frames: int = 5    # 멈춰서 확인 중 판정이 이만큼 연속 사라지면 오판정 → 다시 출발
    verify_timeout_s: float = 2.0   # 확정도 사라짐도 없이 이 시간이 지나면 다시 출발 (판정이 깜빡이는 경우)
    verify_cooldown_s: float = 0.5  # 오판정으로 다시 출발한 뒤 이 시간은 판정으로 멈추지 않음 (같은 자리 반복 방지)
    marker_slow_y: float = 60.0     # 목적지 색 스티커가 이 높이 아래로 보이면 감속
    marker_stop_y: float = 120.0    # 이 높이까지 내려오면 정지해서 좌우 확정 — 높이로 정해 속도와 상관없이 비슷한 자리에 섬
    marker_check_y: float = 90.0    # 확정할 때 이 높이 아래의 스티커만 봄 (건너편 마커 제외)
    marker_check_frames: int = 5    # 이만큼 보고 과반으로 좌우 확정
    marker_settle_s: float = 0.3    # 멈춘 뒤 흔들림이 가라앉을 때까지 기다림
    marker_check_timeout_s: float = 1.5  # 영상이 모자라도 이 시간이면 판단
    marker_ignore_s: float = 2.0    # 직진·회전 뒤 이 시간은 마커 무시 (같은 마커 재인식 방지)
    turn_forward_s: float = 0.7     # 회전 전 전진 시간 — 카메라가 차축보다 앞을 보므로 회전 중심을 분기점 가까이
    turn_speed: float = 0.6         # 제자리 회전 세기
    turn_min_s: float = 0.3         # 회전 시작 후 이 시간은 새 라인으로 인정 안 함
    turn_clear_frames: int = 2      # 기존 라인이 이만큼 연속 안 보이면 새 라인을 찾기 시작
    turn_jump_offset: float = 0.25  # 또는 오프셋이 도는 쪽으로 이만큼 튀면 — 광각이라 기존 라인이 나가기 전에 새 라인이 들어옴
    turn_accept_offset: float = 0.4 # 새 라인 인정: 기준 위치에서 이만큼 안쪽
    turn_accept_deg: float = 35.0   # 새 라인 인정: 90° ± 이 각도 (T자 가로선 제외)
    turn_seen_frames: int = 3       # 새 라인이 이만큼 연속 보이면 회전 끝
    turn_timeout_s: float = 4.0     # 이 시간 안에 못 찾으면 정지·E_LINE
    turn_settle_s: float = 1.0      # 회전 후 이 시간은 정지선·코너·라인 끝을 무시 (분기점 가로선 재검출 방지)


@dataclass
class MarkerInput:
    """marker_node의 /marker/state 한 항목."""
    color: str
    side: str                       # left / right (라인 기준)
    y: float                        # 화면 높이 — 클수록 차에 가까움


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


def merge_stop_line(line: LineInput, detected: bool, confirmed: bool) -> LineInput:
    """/line/state에 /stop_line/state 판정을 합침 — 정지선이 있으면 이벤트를 STOP_LINE으로."""
    event = LineEvent.STOP_LINE if confirmed else line.event
    raw = LineEvent.STOP_LINE if detected or confirmed else line.raw_event
    return LineInput(line.offset, line.line_found, event, raw, line.angle, line.angle_valid)


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
        self.recheck_t = None               # LINE_END 재확인 경과 시간 (None = 재확인 중 아님)
        self.recheck_seen = 0               # 재확인 중 라인이 연속으로 보인 프레임 수
        self.verify_t = None                # 판정 후 멈춰서 확인한 시간 (None = 확인 중 아님)
        self.verify_clear = 0               # 확인 중 판정이 연속으로 없던 프레임 수
        self.verify_event = LineEvent.NONE
        self.cooldown = 0.0                 # 남은 재정지 금지 시간
        self._floor = (self.cfg.min_speed, self.cfg.event_speed)   # set_speed가 낮췄다 올릴 때 되돌릴 값
        self.target = None                  # 목적지 색 (None = 마커 안 봄, /drive/target)
        self.marker_near = False            # 목적지 색 스티커가 보임 → 감속
        self.marker_ignore = 0.0            # 남은 마커 무시 시간
        self.event_ignore = 0.0             # 남은 라인 이벤트 무시 시간 (회전 직후)
        self.check_t = None                 # 스티커 앞에 멈춰 좌우를 확인한 시간 (None = 확인 중 아님)
        self.check_counts = [0, 0, 0]       # 본 프레임 수, 왼쪽에 보인 수, 오른쪽에 보인 수
        self.turn_dir = 0                   # 회전 방향 (+1 왼쪽, -1 오른쪽, 0 = 회전 중 아님)
        self.turn_stage = ""                # FORWARD → LEAVE(기존 라인 벗어나기) → SEEK(새 라인 찾기)
        self.turn_t = 0.0
        self.turn_frames = 0                # LEAVE: 라인이 안 보인 프레임 수, SEEK: 라인이 보인 프레임 수
        self.turn_offset = None             # LEAVE: 직전 라인 오프셋 (돌수록 기존 라인은 반대쪽으로 밀림)

    @property
    def active(self) -> bool:
        return self.command is not None

    @property
    def phase(self) -> str:
        """지금 단계 — 앱 표시용. "FOLLOW", "RECHECK 2/3", "VERIFY STOP_LINE", "MARKER red", "TURN LEFT"."""
        if self.check_t is not None:
            return f"MARKER {self.target}"
        if self.turn_dir:
            return "TURN LEFT" if self.turn_dir > 0 else "TURN RIGHT"
        if self.verify_t is not None:
            return f"VERIFY {self.verify_event.value}"
        if self.recheck_t is None:
            return "FOLLOW"
        c = self.cfg
        return f"RECHECK {min(c.recheck_tries, int(self.recheck_t / c.recheck_s) + 1)}/{c.recheck_tries}"

    def set_speed(self, speed: float) -> None:
        """주행 기본 속도 변경 (노트북 앱, 주행 중에도). 감속 하한·이벤트 감속이 기본 속도보다 크지 않게 맞춘다."""
        self.cfg.speed = speed
        self.cfg.min_speed = min(self._floor[0], speed)
        self.cfg.event_speed = min(self._floor[1], speed)

    def set_tune(self, key: str, value: float) -> None:
        """노트북 앱 조정 (DEV TUNE) — TURN: 회전 세기, CENTER: 라인 기준 위치, FORWARD: 회전 전 전진 시간,
        SPIN: 제자리 회전 세기. 주행 중에도 바로 적용."""
        if key == "TURN":
            self.cfg.turn_scale = value
        elif key == "CENTER":
            self.cfg.center_offset = value
        elif key == "FORWARD":
            self.cfg.turn_forward_s = value
        elif key == "SPIN":
            self.cfg.turn_speed = value
        else:
            raise ValueError(f"모르는 조정 항목: {key}")

    def set_target(self, color: str | None) -> None:
        """따라갈 목적지 색 (None = 마커 안 봄). 지금은 노트북 앱, 나중에 mission_node가 정한다."""
        self.target = color
        self.marker_near = False

    def _clear_phases(self) -> None:
        self.recheck_t = self.verify_t = self.check_t = None
        self.turn_dir = 0
        self.marker_near = False

    def start(self, command: DriveCommand) -> None:
        self.command = DriveCommand(command)
        self.pid.reset()
        self.since_line = 0.0
        self.last = DriveOutput(0.0, 0.0)
        self._clear_phases()
        self.cooldown = self.marker_ignore = self.event_ignore = 0.0

    def cancel(self) -> DriveOutput:
        self.command = None
        self._clear_phases()
        return DriveOutput(0.0, 0.0, True, "STOPPED")

    def step(self, line: LineInput | None, dt: float, markers: list[MarkerInput] | None = None) -> DriveOutput:
        """line: 이번 주기에 새로 받은 라인 상태(없으면 None). markers: 새로 받은 마커 상태(없으면 None). dt: 주기(초)."""
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
            self._clear_phases()
            return DriveOutput(0.0, 0.0, True, "E_LINE")
        if self.recheck_t is not None:
            return self._step_recheck(line, dt)
        if self.verify_t is not None:
            return self._step_verify(line, dt)
        if self.check_t is not None:
            return self._step_check(markers, dt)
        if self.turn_dir:
            return self._step_turn(line, dt)
        self.cooldown = max(0.0, self.cooldown - dt)
        self.marker_ignore = max(0.0, self.marker_ignore - dt)
        self.event_ignore = max(0.0, self.event_ignore - dt)
        if markers is not None and self._on_markers(markers):
            return self.last
        if line is None:                       # 이번 주기엔 새 영상 없음 → 직전 출력 유지
            return self.last
        if self.event_ignore <= 0.0:
            if line.event == LineEvent.LINE_END and self.cfg.recheck_tries > 0:
                self.recheck_t, self.recheck_seen = 0.0, 0     # 멈춰서 다시 봄 — 주행 중 흔들림으로 놓쳤을 수 있음
                self.last = DriveOutput(0.0, 0.0)
                return self.last
            if line.event in STOP_EVENTS:
                self.command = None
                return DriveOutput(0.0, 0.0, True, line.event.value)
            if line.raw_event in VERIFY_EVENTS and self.cooldown <= 0.0:
                self.verify_t, self.verify_clear, self.verify_event = 0.0, 0, line.raw_event   # 일단 멈추고 확인
                self.last = DriveOutput(0.0, 0.0)
                return self.last

        c = self.cfg
        err = line.offset - c.center_offset                 # 라인이 기준 위치보다 오른쪽이면 +
        heading = heading_error(line.angle, c.heading_deadband_deg) / 45.0 if line.angle_valid else 0.0
        angular = _clip(-c.turn_scale * (self.pid.update(err, dt) + c.kh * heading), c.max_angular)
        linear = max(c.min_speed, c.speed * (1.0 - c.slow_gain * abs(err)))
        if line.raw_event != LineEvent.NONE or not line.line_found or self.marker_near:
            linear = min(linear, c.event_speed)          # 이벤트나 목적지 마커가 보이기 시작하면 감속
        self.last = DriveOutput(round(linear, 3), round(angular, 3))
        return self.last

    def _step_recheck(self, line: LineInput | None, dt: float) -> DriveOutput:
        """LINE_END 후 정지한 채 라인을 다시 본다. recheck_frames 연속 보이면 재출발, 모든 횟수가 지나면 진짜 라인 끝."""
        c = self.cfg
        self.recheck_t += dt
        if line is not None:
            self.recheck_seen = self.recheck_seen + 1 if line.line_found else 0
            if self.recheck_seen >= c.recheck_frames:
                self.recheck_t = None
                self.pid.reset()                           # 멈춰 있던 동안의 변화율이 조향에 튀지 않게
                return self.last
        if self.recheck_t >= c.recheck_tries * c.recheck_s:
            self.recheck_t = None
            self.command = None
            return DriveOutput(0.0, 0.0, True, LineEvent.LINE_END.value)
        return self.last

    def _step_verify(self, line: LineInput | None, dt: float) -> DriveOutput:
        """정지선·코너 판정 후 멈춘 채 확인 — 확정되면 끝, 판정이 사라지면(또는 시간 초과) 오판정으로 보고 다시 출발."""
        c = self.cfg
        self.verify_t += dt
        if line is not None:
            if line.event in VERIFY_EVENTS:
                self.verify_t = None
                self.command = None
                return DriveOutput(0.0, 0.0, True, line.event.value)
            self.verify_clear = 0 if line.raw_event in VERIFY_EVENTS else self.verify_clear + 1
        if self.verify_clear >= c.verify_clear_frames or self.verify_t >= c.verify_timeout_s:
            self.verify_t = None
            self.cooldown = c.verify_cooldown_s
            self.pid.reset()
        return self.last

    def _target_sides(self, markers: list[MarkerInput], min_y: float) -> set[str]:
        """min_y보다 아래(가까이)에 보이는 목적지 색 스티커의 좌우."""
        return {m.side for m in markers if m.color == self.target and m.y >= min_y}

    def _on_markers(self, markers: list[MarkerInput]) -> bool:
        """주행 중 마커 상태. 목적지 색이 보이면 감속, 판단 위치까지 오면 정지해서 확인 (True), 양쪽이면 멈추지 않고 직진."""
        c = self.cfg
        if self.target is None or self.marker_ignore > 0.0:
            self.marker_near = False
            return False
        self.marker_near = bool(self._target_sides(markers, c.marker_slow_y))
        sides = self._target_sides(markers, c.marker_stop_y)
        if len(sides) == 2:
            self._resume_after_marker()
            return False
        if sides:
            self.check_t, self.check_counts = 0.0, [0, 0, 0]
            self.marker_near = False
            self.last = DriveOutput(0.0, 0.0)
            return True
        return False

    def _resume_after_marker(self) -> None:
        self.marker_ignore = self.cfg.marker_ignore_s
        self.marker_near = False

    def _step_check(self, markers: list[MarkerInput] | None, dt: float) -> DriveOutput:
        """스티커 앞에 멈춰 좌우 확정 — 왼쪽만 좌회전, 오른쪽만 우회전, 양쪽 직진, 사라지면 오판정으로 보고 다시 출발."""
        c = self.cfg
        self.check_t += dt
        counts = self.check_counts
        if markers is not None and self.check_t >= c.marker_settle_s:
            sides = self._target_sides(markers, c.marker_check_y)
            counts[0] += 1
            counts[1] += "left" in sides
            counts[2] += "right" in sides
        if counts[0] < c.marker_check_frames and self.check_t < c.marker_check_timeout_s:
            return self.last
        self.check_t = None
        need = counts[0] // 2 + 1
        left, right = counts[1] >= need, counts[2] >= need
        self.pid.reset()
        self._resume_after_marker()
        if left != right:
            self.turn_dir, self.turn_stage, self.turn_t, self.turn_frames = (1 if left else -1), "FORWARD", 0.0, 0
        return self.last

    def _line_ahead(self, line: LineInput) -> bool:
        """회전을 끝내도 되는 라인 — 기준 위치 근처에 세로에 가깝게. line_found만 보면 기존 라인·T자 가로선에서 바로 끝남."""
        c = self.cfg
        return (line.line_found and line.angle_valid and abs(line.offset - c.center_offset) <= c.turn_accept_offset
                and abs(line.angle - 90.0) <= c.turn_accept_deg)

    def _step_turn(self, line: LineInput | None, dt: float) -> DriveOutput:
        """전진 → 제자리 회전 → 기존 라인이 벗어난 뒤 새 라인이 보이면 끝. 회전 각도는 재지 않음."""
        c = self.cfg
        self.turn_t += dt
        if self.turn_stage == "FORWARD":
            if self.turn_t < c.turn_forward_s:
                self.last = DriveOutput(c.event_speed, 0.0)
                return self.last
            self.turn_stage, self.turn_t, self.turn_frames, self.turn_offset = "LEAVE", 0.0, 0, None
        if self.turn_t >= c.turn_timeout_s:
            self.command = None
            self._clear_phases()
            return DriveOutput(0.0, 0.0, True, "E_LINE")
        self.last = DriveOutput(0.0, round(self.turn_dir * c.turn_speed, 3))
        if line is None:
            return self.last
        ahead = self._line_ahead(line)
        if self.turn_stage == "LEAVE":
            self.turn_frames = 0 if ahead else self.turn_frames + 1
            jumped = (line.line_found and self.turn_offset is not None
                      and self.turn_dir * (self.turn_offset - line.offset) >= c.turn_jump_offset)
            if line.line_found:
                self.turn_offset = line.offset
            if self.turn_frames < c.turn_clear_frames and not jumped:
                return self.last
            self.turn_stage, self.turn_frames = "SEEK", 0
        self.turn_frames = self.turn_frames + 1 if ahead and self.turn_t >= c.turn_min_s else 0
        if self.turn_frames >= c.turn_seen_frames:
            self.turn_dir = 0
            self.pid.reset()
            self.event_ignore = self.cooldown = c.turn_settle_s
            self.marker_ignore = c.marker_ignore_s
        return self.last

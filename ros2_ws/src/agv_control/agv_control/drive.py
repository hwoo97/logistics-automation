"""주행 제어 (② 순수 로직) — 라인 상태 → /cmd_vel (functional_design F4·F5, class_design 3.1).

지금 구현: FOLLOW_TO_EVENT(라인 따라가다 이벤트에서 정지, skip_stop_lines만큼 정지선 통과),
UTURN(제자리에서 돌아 왔던 라인을 다시 찾음), APPROACH(정지선 뒤 표지판까지 직진 서행 — 카메라가 표지판 쪽으로
올라가 있어 라인을 보지 않음, 멈추는 것은 mission_node가 초음파로 취소), STOP. TURN·STRAIGHT는 쓰지 않음 (마커 회전은 FOLLOW 안에서).
부호: 오프셋 + = 라인이 오른쪽 → 오른쪽으로 돌아야 함 → /cmd_vel z는 음수 (z > 0 = 왼쪽 회전, agv_hw.logic.mix).
각도 보정: 라인 각도가 90° ± heading_deadband_deg(85~95°)를 벗어나면 벗어난 만큼 회전에 더함.
  각도 < 85 (위쪽이 오른쪽으로 기움) → 오른쪽으로, 각도 > 95 → 왼쪽으로.
회전 = −turn_scale × (PID(오프셋 − center_offset) + kh × 각도 오차/45°)
  center_offset: 카메라가 차체 중심에서 비켜 달려 있을 때 라인이 와야 할 화면 위치 (−0.1 = 가운데에서 10% 왼쪽)
  turn_scale   : 회전 세기 — 노트북 앱에서 주행 중 조정 (흔들리면 낮춤)
마커 (목적지 색이 정해졌을 때만): 스티커는 라인 테이프 위 (2026-11-06 변경). marker_node가 center(앞으로 이어지는
  라인 위)·left·right(옆 갈래 위)로 알려 줌. 목적지 색이 center = 직진(멈추지 않음), 왼쪽 갈래만 = 좌회전,
  오른쪽 갈래만 = 우회전, 양쪽 갈래 = 직진(잘못 붙인 경우).
  갈래 스티커가 보이면 감속 → 화면 아래쪽까지 오면 정지해 여러 프레임으로 확정 → 전진 → 제자리 회전.
  한쪽 스티커라도 그쪽에 라인 갈래(line_node branch_left/right)가 안 보이면 회전하지 않고 직진 (2026-11-06 실차:
  직진·좌회전 갈래뿐인 곳에서 스티커가 오른쪽 → 오른쪽으로 돌다 왔던 라인을 잡아 유턴이 됨).
  회전은 각도가 아니라 라인으로 끝낸다 (엔코더 없음): 기존 라인이 벗어난 뒤, 새 라인이 기준 근처에 세로에 가깝게 보이면 끝.
"""
from dataclasses import dataclass, replace

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
    marker_branch_check: bool = True    # 스티커 쪽에 갈래가 안 보이면 직진 (끄면 스티커만 보고 회전)
    marker_ignore_s: float = 2.0    # 직진·회전 뒤 이 시간은 마커 무시 (같은 마커 재인식 방지)
    turn_forward_s: float = 0.7     # 회전 전 전진 시간 — 카메라가 차축보다 앞을 보므로 회전 중심을 분기점 가까이
    turn_speed: float = 0.6         # 제자리 회전 세기
    turn_blind_s: float = 2.0       # 마커 회전 시작 후 이 시간은 라인을 아예 보지 않고 돌기만 함 (기존 라인·분기점 가로선에
                                    # 걸리지 않게, 앱 "회전 중 라인 무시"). 끝나면 바로 새 라인 찾기. 0 = 끄기(기존 방식)
    turn_min_s: float = 0.3         # 회전 시작 후 이 시간은 새 라인으로 인정 안 함
    turn_clear_frames: int = 2      # 기존 라인이 이만큼 연속 안 보이면 새 라인을 찾기 시작
    turn_jump_offset: float = 0.25  # 또는 오프셋이 도는 쪽으로 이만큼 튀면 — 광각이라 기존 라인이 나가기 전에 새 라인이 들어옴
    turn_accept_offset: float = 0.4 # 새 라인 인정: 기준 위치에서 이만큼 안쪽
    turn_accept_deg: float = 35.0   # 새 라인 인정: 90° ± 이 각도 (T자 가로선 제외)
    turn_seen_frames: int = 3       # 새 라인이 이만큼 연속 보이면 회전 끝
    turn_timeout_s: float = 4.0     # 이 시간 안에 못 찾으면 정지·E_LINE (라인 무시 시간이 끝난 뒤부터 셈)
    turn_settle_s: float = 1.0      # 회전 후 이 시간은 정지선·코너·라인 끝을 무시 (분기점 가로선 재검출 방지)
    uturn_min_s: float = 1.5        # 유턴 시작 후 이 시간은 새 라인으로 인정 안 함 (정지선·옆 갈래에서 멈추지 않게)
    uturn_timeout_s: float = 8.0    # 이 시간 안에 라인을 못 찾으면 정지·E_TURN
    uturn_dir: int = -1             # 유턴 방향: -1 = 오른쪽(위에서 봐서 시계 방향), +1 = 왼쪽(반시계) — 2026-11-09 사용자:
                                    # 하드웨어가 좌회전에 약해 시계 방향으로 시험 (이전 고정값 +1)
    # 유턴 후 맞추기 (2026-11-06 실차: 빠르게 돌다 멈춰 라인이 화면 끝에 남음 → 다음 출발에서 라인을 놓침)
    uturn_align: bool = True        # 라인을 찾은 뒤 천천히 돌려 라인을 기준 위치에 맞추고 끝냄 (false = 찾자마자 끝)
    align_tol: float = 0.1          # 기준 위치(center_offset)에서 이만큼 안이면 맞음
    align_frames: int = 2           # 멈춘 채 이만큼 연속 맞으면 끝
    align_gain: float = 1.0         # 회전 = 오차 × gain, 아래 범위로 자름
    align_min: float = 0.3          # 최소 회전 세기 — 이보다 약하면 모터가 안 돎
    align_max: float = 0.45         # 최대 회전 세기 — 빠르면 다시 지나침
    align_timeout_s: float = 3.0    # 이 시간 안에 못 맞추면 그대로 끝 (UTURN_DONE)
    approach_speed: float = 0.35    # 표지판 접근 속도 (앱 "접근 속도", DEV TUNE APPROACH) — 2026-11-06 실차 0.45가 빠름
    approach_kick_s: float = 0.3    # 출발할 때 이 시간은 시동 세기로 — TT 모터는 멈춘 상태에서 낮은 값으로는 안 움직임
    approach_kick_speed: float = 0.45   # 시동 세기 (접근 속도가 이보다 높으면 접근 속도 그대로)
    # 초음파 정지 (2026-11-06) — 라인 주행(FOLLOW_TO_EVENT) 중 가까운 물체에서 멈춤 → 결과 OBSTACLE.
    # 제자리 회전(마커 회전·유턴)과 APPROACH(mission_node가 따로 멈춤)에서는 보지 않음
    obstacle_stop: bool = True
    obstacle_distance: float = 0.18 # 이 거리(m) 이하가
    obstacle_hits: int = 3          # 이만큼 연속이면 정지 (값 없음은 건너뜀)
    pass_clear_frames: int = 8      # 정지선 통과: 판정이 이만큼 연속 사라지면 지나간 것으로 봄
    pass_window_s: float = 3.0      # 출발 후 이 시간 안에 통과할 정지선이 안 보이면 이미 지난 것으로 보고 통과 횟수를 0으로
                                    # (2026-11-09: 출발 때 정지선이 카메라 아래 사각지대에 있으면 못 보고, 남은 1이
                                    # 목적지 정지선을 지워 지나침). 0 = 끄기


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
    branch_left: bool = False       # 라인과 이어진 왼쪽 갈래가 보임
    branch_right: bool = False


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
    return replace(line, event=event, raw_event=raw)


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
        self.note, self.note_no = "", 0     # 정지선 처리 기록 (drive_node가 로그로 — 지나친 이유 찾기)
        self._noted = ""                    # 같은 무시 이유를 반복해서 남기지 않게
        self.check_t = None                 # 스티커 앞에 멈춰 좌우를 확인한 시간 (None = 확인 중 아님)
        self.check_counts = [0, 0, 0]       # 본 프레임 수, 왼쪽에 보인 수, 오른쪽에 보인 수
        self.branch_counts = [0, 0, 0]      # 확인 중 라인 프레임 수, 왼쪽 갈래 보인 수, 오른쪽 갈래 보인 수
        self.decision = ""                  # 마지막 마커 판단 (로그용) "LEFT", "STRAIGHT (오른쪽 스티커, 오른쪽 갈래 없음)" 등
        self.decision_no = 0                # 판단할 때마다 1 증가 — drive_node가 바뀐 것을 보고 로그
        self.turn_dir = 0                   # 회전 방향 (+1 왼쪽, -1 오른쪽, 0 = 회전 중 아님)
        self.turn_stage = ""                # FORWARD → BLIND(라인 안 봄) 또는 LEAVE(기존 라인 벗어나기) → SEEK(새 라인 찾기)
        self.turn_t = 0.0
        self.turn_frames = 0                # LEAVE: 라인이 안 보인 프레임 수, SEEK: 라인이 보인 프레임 수
        self.turn_offset = None             # LEAVE: 직전 라인 오프셋 (돌수록 기존 라인은 반대쪽으로 밀림)
        self.range_hits = 0                 # 초음파가 obstacle_distance 이하로 연속 나온 수
        self.approach_t = 0.0               # APPROACH 시작 후 시간 (시동 구간)
        self.skip_stop = 0                  # 남은 통과할 정지선 수 (유턴 뒤 1 — 방금 지점의 정지선)
        self.passing = False                # 지금 정지선 위를 지나가는 중
        self.pass_clear = 0                 # 통과 중 정지선 판정이 연속으로 없던 프레임 수
        self.pass_t = 0.0                   # 출발 후 라인 주행 시간 (통과할 정지선을 기다린 시간)
        self.pass_seen = False              # 통과할 정지선을 한 번이라도 봤는지

    @property
    def active(self) -> bool:
        return self.command is not None

    @property
    def phase(self) -> str:
        """지금 단계 — 앱 표시용. "FOLLOW", "RECHECK 2/3", "VERIFY STOP_LINE", "MARKER red", "TURN LEFT", "UTURN"."""
        if self.command in (DriveCommand.UTURN, DriveCommand.APPROACH):
            return self.command.value
        if self.check_t is not None:
            return f"MARKER {self.target}"
        if self.turn_dir:
            return "TURN LEFT" if self.turn_dir > 0 else "TURN RIGHT"
        if self.verify_t is not None:
            return f"VERIFY {self.verify_event.value}"
        if self.passing:
            return "PASS STOP_LINE"
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
        SPIN: 제자리 회전 세기, BLIND: 마커 회전 중 라인 무시 시간, APPROACH: 표지판 접근 속도. 주행 중에도 바로 적용."""
        if key == "TURN":
            self.cfg.turn_scale = value
        elif key == "CENTER":
            self.cfg.center_offset = value
        elif key == "FORWARD":
            self.cfg.turn_forward_s = value
        elif key == "SPIN":
            self.cfg.turn_speed = value
        elif key == "BLIND":
            self.cfg.turn_blind_s = value
        elif key == "APPROACH":
            self.cfg.approach_speed = value
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

    def start(self, command: DriveCommand, skip_stop_lines: int = 0) -> None:
        self.command = DriveCommand(command)
        self.skip_stop, self.passing, self.pass_clear = int(skip_stop_lines), False, 0
        self.pass_t, self.pass_seen = 0.0, False
        self.pid.reset()
        self.since_line = 0.0
        self.last = DriveOutput(0.0, 0.0)
        self._clear_phases()
        self.cooldown = self.marker_ignore = self.event_ignore = 0.0
        self.range_hits = 0
        self.approach_t = 0.0
        if self.command == DriveCommand.UTURN:          # uturn_dir 쪽으로 제자리 회전 — 전진 없음 (정지선 너머로 가지 않게)
            udir = -1 if self.cfg.uturn_dir < 0 else 1
            self.turn_dir, self.turn_stage, self.turn_t, self.turn_frames, self.turn_offset = udir, "LEAVE", 0.0, 0, None

    def _watch_range(self) -> bool:
        """초음파 정지를 볼 때 — 라인 주행 중, 제자리 회전이 아닐 때 (회전 전 전진은 봄)."""
        return (self.cfg.obstacle_stop and self.command == DriveCommand.FOLLOW_TO_EVENT
                and not (self.turn_dir and self.turn_stage != "FORWARD"))

    def set_range(self, distance: float | None) -> None:
        """초음파 거리(m, 값 없음 = None). 볼 때가 아니면 연속 수를 지움."""
        if not self._watch_range():
            self.range_hits = 0
        elif distance is not None:
            self.range_hits = self.range_hits + 1 if distance <= self.cfg.obstacle_distance else 0

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
        if self.command not in (DriveCommand.FOLLOW_TO_EVENT, DriveCommand.UTURN, DriveCommand.APPROACH):
            name, self.command = self.command.value, None
            return DriveOutput(0.0, 0.0, True, f"E_UNSUPPORTED_{name}")
        if self.range_hits >= self.cfg.obstacle_hits and self._watch_range():
            self.command = None
            self._clear_phases()
            self.range_hits = 0
            return DriveOutput(0.0, 0.0, True, "OBSTACLE")

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
            return self._step_check(markers, dt, line)
        if self.turn_dir:
            return self._step_turn(line, dt)
        if self.command == DriveCommand.APPROACH:
            return self._step_approach(line, dt)
        self.cooldown = max(0.0, self.cooldown - dt)
        self.marker_ignore = max(0.0, self.marker_ignore - dt)
        self.event_ignore = max(0.0, self.event_ignore - dt)
        self.pass_t += dt
        if markers is not None and self._on_markers(markers):
            return self.last
        if line is None:                       # 이번 주기엔 새 영상 없음 → 직전 출력 유지
            return self.last
        line = self._pass_stop_line(line)
        self._note_ignored(line)
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
                self._note(f"{line.raw_event.value} 보임 → 멈추고 확인")
                self.last = DriveOutput(0.0, 0.0)
                return self.last

        c = self.cfg
        err = line.offset - c.center_offset                 # 라인이 기준 위치보다 오른쪽이면 +
        angular = self._steer(line, dt)
        linear = max(c.min_speed, c.speed * (1.0 - c.slow_gain * abs(err)))
        if line.raw_event != LineEvent.NONE or not line.line_found or self.marker_near:
            linear = min(linear, c.event_speed)          # 이벤트나 목적지 마커가 보이기 시작하면 감속
        self.last = DriveOutput(round(linear, 3), angular)
        return self.last

    def _note(self, text: str) -> None:
        self.note, self.note_no = text, self.note_no + 1

    def _note_ignored(self, line: LineInput) -> None:
        """정지선이 보였는데 멈추지 않는 이유를 한 번씩 기록 (회전 직후 무시, 오판정 뒤 쉬는 시간)."""
        if LineEvent.STOP_LINE not in (line.event, line.raw_event):
            self._noted = ""
            return
        why = ""
        if self.event_ignore > 0.0:
            why = f"회전 직후 무시 중 (남은 {self.event_ignore:.1f}초)"
        elif self.cooldown > 0.0:
            why = f"오판정 뒤 쉬는 중 (남은 {self.cooldown:.1f}초)"
        if why and why.split(" (")[0] != self._noted:
            self._noted = why.split(" (")[0]
            self._note(f"정지선 보였지만 멈추지 않음 — {why}")

    def _pass_stop_line(self, line: LineInput) -> LineInput:
        """통과할 정지선이 남아 있으면 정지선 판정을 지우고 지나간다. 판정이 연속으로 사라지면 하나 지난 것."""
        if self.skip_stop <= 0:
            return line
        seen = LineEvent.STOP_LINE in (line.event, line.raw_event)
        c = self.cfg
        if not seen and not self.pass_seen and c.pass_window_s > 0 and self.pass_t >= c.pass_window_s:
            self.skip_stop, self.passing = 0, False             # 출발 때 이미 카메라 밖 — 목적지 정지선은 멈춰야 함
            self._note(f"통과할 정지선이 {c.pass_window_s:g}초 동안 안 보임 → 이미 지난 것으로 봄 (다음 정지선에서 멈춤)")
            return line
        if seen:
            self.passing, self.pass_clear, self.pass_seen = True, 0, True
        elif self.passing:
            self.pass_clear += 1
            if self.pass_clear >= self.cfg.pass_clear_frames:
                self.skip_stop -= 1
                self.passing = False
                self._note(f"정지선 하나 통과 (지시대로, 남은 통과 {self.skip_stop})")
        if not seen:
            return line
        drop = lambda e: LineEvent.NONE if e == LineEvent.STOP_LINE else e   # noqa: E731
        return replace(line, event=drop(line.event), raw_event=drop(line.raw_event))

    def _steer(self, line: LineInput, dt: float) -> float:
        c = self.cfg
        err = line.offset - c.center_offset
        heading = heading_error(line.angle, c.heading_deadband_deg) / 45.0 if line.angle_valid else 0.0
        return round(_clip(-c.turn_scale * (self.pid.update(err, dt) + c.kh * heading), c.max_angular), 3)

    def _step_approach(self, line: LineInput | None, dt: float) -> DriveOutput:
        """표지판까지 직진 서행 — 카메라가 표지판 쪽을 보고 있으므로(교수 변경 시나리오 순서) 라인·이벤트·마커를 모두 무시.
        라인 상태는 영상이 오고 있는지(line_timeout)만 확인하는 데 씀. 처음 approach_kick_s는 시동 세기로 출발."""
        c = self.cfg
        speed = c.approach_speed if self.approach_t >= c.approach_kick_s else max(c.approach_speed, c.approach_kick_speed)
        self.approach_t += dt
        self.last = DriveOutput(round(speed, 3), 0.0)
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
                self._note(f"{line.event.value} 확정 ({self.verify_t:.1f}초)")
                self.verify_t = None
                self.command = None
                return DriveOutput(0.0, 0.0, True, line.event.value)
            self.verify_clear = 0 if line.raw_event in VERIFY_EVENTS else self.verify_clear + 1
        if self.verify_clear >= c.verify_clear_frames or self.verify_t >= c.verify_timeout_s:
            why = (f"판정 {c.verify_clear_frames}장 연속 사라짐" if self.verify_clear >= c.verify_clear_frames
                   else f"{c.verify_timeout_s:g}초 동안 확정 안 됨")
            self._note(f"{self.verify_event.value} 확인 실패 — {why} → 오판정으로 보고 다시 출발")
            self.verify_t = None
            self.cooldown = c.verify_cooldown_s
            self.pid.reset()
        return self.last

    def _decide(self, text: str) -> None:
        self.decision, self.decision_no = text, self.decision_no + 1

    def _target_sides(self, markers: list[MarkerInput], min_y: float) -> set[str]:
        """min_y보다 아래(가까이)에 보이는 목적지 색 스티커의 위치 (left / center / right)."""
        return {m.side for m in markers if m.color == self.target and m.y >= min_y}

    def _on_markers(self, markers: list[MarkerInput]) -> bool:
        """주행 중 마커 상태. 갈래 위 목적지 색이 보이면 감속, 판단 위치까지 오면 정지해서 확인 (True).
        앞 라인 위(center)거나 양쪽 갈래면 멈추지 않고 직진."""
        c = self.cfg
        if self.target is None or self.marker_ignore > 0.0:
            self.marker_near = False
            return False
        self.marker_near = bool(self._target_sides(markers, c.marker_slow_y) & {"left", "right"})
        sides = self._target_sides(markers, c.marker_stop_y)
        turn = sides & {"left", "right"}
        if len(turn) == 2:
            self._resume_after_marker()
            self._decide("STRAIGHT (양쪽 갈래 스티커)")
            return False
        if not turn and "center" in sides:
            self._resume_after_marker()
            self._decide("STRAIGHT (앞 라인 위 마커)")
            return False
        if turn:
            self.check_t, self.check_counts, self.branch_counts = 0.0, [0, 0, 0], [0, 0, 0]
            self.marker_near = False
            self.last = DriveOutput(0.0, 0.0)
            return True
        return False

    def _resume_after_marker(self) -> None:
        self.marker_ignore = self.cfg.marker_ignore_s
        self.marker_near = False

    def _step_check(self, markers: list[MarkerInput] | None, dt: float, line: LineInput | None = None) -> DriveOutput:
        """스티커 앞에 멈춰 좌우 확정 — 왼쪽만 좌회전, 오른쪽만 우회전, 양쪽 직진, 사라지면 오판정으로 보고 다시 출발.
        한쪽 스티커인데 그쪽 갈래가 과반 프레임에서 안 보이면 직진."""
        c = self.cfg
        self.check_t += dt
        counts, branch = self.check_counts, self.branch_counts
        if markers is not None and self.check_t >= c.marker_settle_s:
            sides = self._target_sides(markers, c.marker_check_y)
            counts[0] += 1
            counts[1] += "left" in sides
            counts[2] += "right" in sides
        if line is not None and line.line_found and self.check_t >= c.marker_settle_s:
            branch[0] += 1
            branch[1] += line.branch_left
            branch[2] += line.branch_right
        if counts[0] < c.marker_check_frames and self.check_t < c.marker_check_timeout_s:
            return self.last
        self.check_t = None
        need = counts[0] // 2 + 1
        left, right = counts[1] >= need, counts[2] >= need
        self.pid.reset()
        self._resume_after_marker()
        if left == right:
            self._decide("STRAIGHT (양쪽 스티커)" if left else "STRAIGHT (스티커 사라짐)")
            return self.last
        name = "왼쪽" if left else "오른쪽"
        if c.marker_branch_check and branch[0] > 0 and branch[1 if left else 2] < branch[0] // 2 + 1:
            self._decide(f"STRAIGHT ({name} 스티커, {name} 갈래 없음)")
            return self.last
        self._decide("LEFT" if left else "RIGHT")
        self.turn_dir, self.turn_stage, self.turn_t, self.turn_frames = (1 if left else -1), "FORWARD", 0.0, 0
        return self.last

    def _finish_uturn(self) -> DriveOutput:
        self.command = None
        self._clear_phases()
        return DriveOutput(0.0, 0.0, True, "UTURN_DONE")

    def _step_align(self, line: LineInput | None) -> DriveOutput:
        """유턴 후 맞추기 — 라인 오프셋을 기준 위치로 천천히 돌려 맞추고, 멈춘 채 연속 맞으면 끝.
        지나쳐서 반대쪽에 있으면 되돌아 돎. 라인이 안 보이면 마지막으로 본 쪽으로 돎."""
        c = self.cfg
        if self.turn_t >= c.align_timeout_s:
            return self._finish_uturn()
        if line is None:
            return self.last
        if line.line_found:
            self.turn_offset = line.offset
            err = line.offset - c.center_offset
            if abs(err) <= c.align_tol:
                self.turn_frames += 1
                if self.turn_frames >= c.align_frames:
                    return self._finish_uturn()
                self.last = DriveOutput(0.0, 0.0)                # 멈춘 채 확인
                return self.last
            self.turn_frames = 0
            w = _clip(c.align_gain * abs(err), c.align_max)
            self.last = DriveOutput(0.0, round(-(1 if err > 0 else -1) * max(c.align_min, w), 3))   # 오른쪽(+)이면 오른쪽으로
            return self.last
        self.turn_frames = 0
        side = (self.turn_dir or 1) if self.turn_offset is None else (-1 if self.turn_offset - c.center_offset > 0 else 1)
        self.last = DriveOutput(0.0, round(side * c.align_min, 3))
        return self.last

    def _line_ahead(self, line: LineInput) -> bool:
        """회전을 끝내도 되는 라인 — 기준 위치 근처에 세로에 가깝게. line_found만 보면 기존 라인·T자 가로선에서 바로 끝남."""
        c = self.cfg
        return (line.line_found and line.angle_valid and abs(line.offset - c.center_offset) <= c.turn_accept_offset
                and abs(line.angle - 90.0) <= c.turn_accept_deg)

    def _step_turn(self, line: LineInput | None, dt: float) -> DriveOutput:
        """전진 → 제자리 회전 → 기존 라인이 벗어난 뒤 새 라인이 보이면 끝. 회전 각도는 재지 않음.
        마커 회전은 turn_blind_s 동안 라인을 보지 않고 돌기만 한 뒤(BLIND) 바로 새 라인을 찾는다(SEEK).
        유턴은 전진 없이 같은 방식으로 돌고(라인 무시 없음), 라인을 찾으면 천천히 돌려 기준 위치에 맞춘 뒤(ALIGN) 끝낸다."""
        c = self.cfg
        uturn = self.command == DriveCommand.UTURN
        self.turn_t += dt
        if self.turn_stage == "FORWARD":
            if self.turn_t < c.turn_forward_s:
                self.last = DriveOutput(c.event_speed, 0.0)
                return self.last
            blind = c.turn_blind_s > 0
            self.turn_stage, self.turn_t, self.turn_frames, self.turn_offset = ("BLIND" if blind else "LEAVE"), 0.0, 0, None
        if self.turn_stage == "BLIND":
            self.last = DriveOutput(0.0, round(self.turn_dir * c.turn_speed, 3))
            if self.turn_t < c.turn_blind_s:
                return self.last
            self.turn_stage, self.turn_t, self.turn_frames = "SEEK", 0.0, 0   # 기존 라인은 벗어났다고 봄, 시간 제한은 여기부터
            return self.last
        if self.turn_stage == "ALIGN":
            return self._step_align(line)
        if self.turn_t >= (c.uturn_timeout_s if uturn else c.turn_timeout_s):
            self.command = None
            self._clear_phases()
            return DriveOutput(0.0, 0.0, True, "E_TURN" if uturn else "E_LINE")
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
        min_s = c.uturn_min_s if uturn else c.turn_min_s
        self.turn_frames = self.turn_frames + 1 if ahead and self.turn_t >= min_s else 0
        if line.line_found:
            self.turn_offset = line.offset
        if self.turn_frames >= c.turn_seen_frames:
            if uturn:
                if c.uturn_align:
                    self.turn_stage, self.turn_t, self.turn_frames = "ALIGN", 0.0, 0
                    return self._step_align(line)
                return self._finish_uturn()
            self.turn_dir = 0
            self.pid.reset()
            self.event_ignore = self.cooldown = c.turn_settle_s
            self.marker_ignore = c.marker_ignore_s
        return self.last

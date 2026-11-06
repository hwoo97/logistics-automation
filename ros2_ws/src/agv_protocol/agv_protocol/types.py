"""여러 패키지(vision·control·bt·노트북 앱)가 함께 쓰는 이름 — class_design.md 0절.

ROS 메시지·BT 메시지에는 `.value` 문자열로 담는다.
"""
from enum import Enum


class Target(str, Enum):          # 목적지
    WORK = "WORK"
    DROP_A = "DROP_A"
    DROP_B = "DROP_B"
    PARK = "PARK"


class Cargo(str, Enum):
    NONE = "NONE"
    CIRCLE = "CIRCLE"
    SQUARE = "SQUARE"


class Sign(str, Enum):            # 표지판 인식 결과
    WORK = "sign_work"
    DROP_A = "sign_drop_a"
    DROP_B = "sign_drop_b"
    PARK = "sign_park"


class LineEvent(str, Enum):
    NONE = "NONE"
    STOP_LINE = "STOP_LINE"
    JUNCTION_T = "JUNCTION_T"
    JUNCTION_LEFT = "JUNCTION_LEFT"
    JUNCTION_RIGHT = "JUNCTION_RIGHT"
    CORNER_LEFT = "CORNER_LEFT"
    CORNER_RIGHT = "CORNER_RIGHT"
    LINE_END = "LINE_END"


class DriveCommand(str, Enum):
    FOLLOW_TO_EVENT = "FOLLOW_TO_EVENT"
    TURN_LEFT = "TURN_LEFT"
    TURN_RIGHT = "TURN_RIGHT"
    STRAIGHT = "STRAIGHT"
    UTURN = "UTURN"
    APPROACH = "APPROACH"
    STOP = "STOP"


class Pose(str, Enum):            # 팬틸트 자세
    LINE = "LINE"
    JUNCTION = "JUNCTION"
    SIGN = "SIGN"


class LedState(str, Enum):
    OFF = "OFF"
    RED = "RED"
    GREEN = "GREEN"
    ERROR = "ERROR"


class State(str, Enum):           # 상태 머신 상태 (BT STATUS에 그대로 표시)
    IDLE = "IDLE"
    NAVIGATE = "NAVIGATE"
    CHECK_SIGN = "CHECK_SIGN"
    APPROACH = "APPROACH"
    LOAD = "LOAD"
    UNLOAD = "UNLOAD"
    UTURN = "UTURN"
    PARKED = "PARKED"
    PAUSED_OBSTACLE = "PAUSED_OBSTACLE"
    ERROR = "ERROR"
    ESTOP = "ESTOP"
    MANUAL = "MANUAL"


class Mode(str, Enum):            # 상태 머신 모드 (2026-11-02)
    NONE = "NONE"
    SETUP = "SETUP"               # 초기 설정 — 색마다 지점을 찾아가 트랙 매핑
    WORK = "WORK"                 # 작업 — 모양별 상차·하차


COLORS = ("red", "green", "blue", "yellow")      # 마커 색 = 초기 설정 탐색 순서. 지점 ↔ 색은 트랙 매핑으로 정함
MAX_ITEMS = 9                                    # 1회 적재 최대 개수 (7세그먼트 1자리) — 넘으면 여러 번 왕복
MAX_ORDER = 99                                   # 물품별 입력 수량 최대 (두 자리, 2026-11-03 변경 시나리오)
CARGO_TARGET = {Cargo.CIRCLE: Target.DROP_A, Cargo.SQUARE: Target.DROP_B}
CARGO_LED = {Cargo.CIRCLE: LedState.RED, Cargo.SQUARE: LedState.GREEN}
SIGN_TARGET = {Sign.WORK: Target.WORK, Sign.DROP_A: Target.DROP_A, Sign.DROP_B: Target.DROP_B, Sign.PARK: Target.PARK}

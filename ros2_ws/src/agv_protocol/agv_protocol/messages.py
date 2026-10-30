"""블루투스 채널 1 텍스트 메시지 — bluetooth_protocol.md 2·3·3-1절.

한 줄 = 메시지 1개. 이 모듈의 함수는 줄 끝 `\\n` 없이 문자열을 만들고 해석한다(전송 계층이 붙이고 뗀다).
차량(bt_bridge_node)과 노트북 앱이 같은 코드를 쓴다.
"""
from dataclasses import dataclass, field

from .types import LedState, Pose, State

CONTROL_CHANNEL = 1
VIDEO_CHANNEL = 2
PING_INTERVAL_S = 0.5
HEARTBEAT_TIMEOUT_S = 1.5
MOVE_REPEAT_S = 0.3

ALL_STATES = frozenset(s.value for s in State)
_HW_STATES = frozenset({"IDLE", "MANUAL", "ESTOP", "ERROR"})

# 명령별 허용 상태 (bluetooth_protocol.md 2절·3-1절)
ALLOWED_STATES = {
    "START": frozenset({"IDLE"}),
    "STOP": ALL_STATES,
    "RESUME": frozenset({"ESTOP", "ERROR"}),
    "MANUAL ON": frozenset({"IDLE", "ESTOP", "ERROR"}),
    "MANUAL OFF": frozenset({"MANUAL"}),
    "MOVE": frozenset({"MANUAL"}),
    "PING": ALL_STATES,
    "DEV LED": _HW_STATES,
    "DEV POSE": _HW_STATES,
    "DEV SERVO": _HW_STATES,
    "DEV WHEEL": frozenset({"IDLE", "MANUAL"}),
    "DEV SPEED": ALL_STATES,              # 주행 중에도 속도 조절
    "DEV DRIVE": ALL_STATES,              # STOP은 언제나, FOLLOW는 IDLE에서만 (check_allowed)
    "DEV TUNE": ALL_STATES,               # 주행 중에도 조정
    "DEV WATCH": ALL_STATES,
    "DEV NODES": ALL_STATES,
}

WATCH_KEYS = ("RANGE", "LINE", "SYS")
SPEED_RANGE = (0.2, 1.0)        # DEV SPEED — drive_node 기본 선속도
DRIVE_ACTIONS = ("FOLLOW", "STOP")      # DEV DRIVE — 시험용 라인 주행 시작·정지
TUNE_RANGES = {"TURN": (0.1, 1.5),      # DEV TUNE — 회전 세기 (회전량 전체에 곱함)
               "CENTER": (-0.5, 0.5)}   #            라인 기준 위치 (오프셋 목표, 음수 = 화면 왼쪽)


@dataclass
class Command:
    kind: str                       # START / STOP / RESUME / MANUAL / MOVE / PING / DEV
    args: list = field(default_factory=list)

    @property
    def key(self) -> str:
        """허용 상태 표의 키 (예: 'MANUAL ON', 'DEV LED')."""
        if self.kind in ("MANUAL", "DEV") and self.args:
            return f"{self.kind} {self.args[0]}"
        return self.kind

    def line(self) -> str:
        return " ".join([self.kind, *self.args])


def _number(text: str, lo: float, hi: float, name: str) -> float:
    try:
        value = float(text)
    except ValueError:
        raise ValueError(f"{name}: 숫자가 아님 ({text})") from None
    if not lo <= value <= hi:
        raise ValueError(f"{name}: {lo}~{hi} 범위 밖 ({text})")
    return value


def _expect_args(cmd: Command, count: int) -> None:
    if len(cmd.args) != count:
        raise ValueError(f"{cmd.key}: 인자 {count}개 필요 (받음 {len(cmd.args)}개)")


def _validate_dev(cmd: Command) -> None:
    if not cmd.args:
        raise ValueError("DEV: 하위 명령 필요")
    sub, rest = cmd.args[0], cmd.args[1:]
    if sub == "LED":
        if len(rest) != 1 or rest[0] not in LedState.__members__:
            raise ValueError(f"DEV LED: {'/'.join(LedState.__members__)} 중 하나")
    elif sub == "POSE":
        if len(rest) != 1 or rest[0] not in Pose.__members__:
            raise ValueError(f"DEV POSE: {'/'.join(Pose.__members__)} 중 하나")
    elif sub == "SERVO":
        if len(rest) != 2:
            raise ValueError("DEV SERVO: pan tilt 2개 필요")
        _number(rest[0], 0, 180, "pan")
        _number(rest[1], 0, 180, "tilt")
    elif sub == "WHEEL":
        if len(rest) != 4:
            raise ValueError("DEV WHEEL: 바퀴 4개 듀티 필요")
        for i, v in enumerate(rest, 1):
            _number(v, -1.0, 1.0, f"L{i}")
    elif sub == "SPEED":
        if len(rest) != 1:
            raise ValueError("DEV SPEED: 주행 기본 속도 1개 필요 (0.2~1.0)")
        _number(rest[0], SPEED_RANGE[0], SPEED_RANGE[1], "speed")
    elif sub == "DRIVE":
        if len(rest) != 1 or rest[0].upper() not in DRIVE_ACTIONS:
            raise ValueError(f"DEV DRIVE: {'/'.join(DRIVE_ACTIONS)} 중 하나")
        cmd.args[1] = rest[0].upper()                 # rest는 복사본 — 원본 인자를 바꿈
    elif sub == "TUNE":
        if len(rest) != 2 or rest[0].upper() not in TUNE_RANGES:
            raise ValueError(f"DEV TUNE: <{'|'.join(TUNE_RANGES)}> <값>")
        cmd.args[1] = rest[0].upper()
        _number(rest[1], *TUNE_RANGES[cmd.args[1]], cmd.args[1].lower())
    elif sub == "WATCH":
        if len(rest) != 2 or rest[0] not in (*WATCH_KEYS, "ALL") or rest[1] not in ("ON", "OFF"):
            raise ValueError("DEV WATCH: <RANGE|LINE|SYS|ALL> <ON|OFF>")
    elif sub == "NODES":
        if rest:
            raise ValueError("DEV NODES: 인자 없음")
    else:
        raise ValueError(f"DEV: 알 수 없는 하위 명령 ({sub})")


def parse_command(line: str) -> Command:
    """노트북 → 차량 명령 해석. 형식 오류면 ValueError (메시지는 NAK 이유로 사용)."""
    parts = line.strip().split()
    if not parts:
        raise ValueError("빈 명령")
    cmd = Command(parts[0].upper(), parts[1:])
    if cmd.kind == "DEV" and cmd.args:
        cmd.args[0] = cmd.args[0].upper()
    if cmd.kind == "START":
        _expect_args(cmd, 1)
        _number(cmd.args[0], 1, 9, "작업 횟수")
        if not cmd.args[0].isdigit():
            raise ValueError(f"작업 횟수: 정수 필요 ({cmd.args[0]})")
    elif cmd.kind in ("STOP", "RESUME", "PING"):
        _expect_args(cmd, 0)
    elif cmd.kind == "MANUAL":
        if len(cmd.args) != 1 or cmd.args[0].upper() not in ("ON", "OFF"):
            raise ValueError("MANUAL: ON 또는 OFF")
        cmd.args[0] = cmd.args[0].upper()
    elif cmd.kind == "MOVE":
        _expect_args(cmd, 2)
        _number(cmd.args[0], -1.0, 1.0, "v")
        _number(cmd.args[1], -1.0, 1.0, "w")
    elif cmd.kind == "DEV":
        _validate_dev(cmd)
    else:
        raise ValueError(f"알 수 없는 명령 ({cmd.kind})")
    return cmd


def command_key(line: str) -> str:
    """형식이 틀린 줄에서도 NAK에 쓸 명령 키를 뽑는다 ('DEV LED BLUE' → 'DEV LED', 'FOO 1' → 'FOO')."""
    parts = line.strip().upper().split()
    if not parts:
        return ""
    two = " ".join(parts[:2])
    return two if two in ALLOWED_STATES else parts[0]


def check_allowed(cmd: Command, state: str) -> str | None:
    """현재 상태에서 받을 수 있으면 None, 아니면 NAK 이유."""
    allowed = ALLOWED_STATES.get(cmd.key, frozenset())
    if cmd.key == "DEV DRIVE" and cmd.args[1:] == ["FOLLOW"] and state != "IDLE":
        return f"{state} 상태에서 불가 (라인 주행 시작은 IDLE에서만)"
    if state in allowed:
        return None
    return f"{state} 상태에서 불가"


def _fmt(value) -> str:
    if isinstance(value, float):
        return f"{value:.3f}".rstrip("0").rstrip(".") if value != int(value) else str(int(value))
    return str(getattr(value, "value", value))


def format_command(kind: str, *args) -> str:
    """노트북 쪽: format_command('START', 3) → 'START 3'. 검증 후 반환."""
    line = " ".join([kind, *(_fmt(a) for a in args)])
    parse_command(line)
    return line


# ── 차량 → 노트북 ─────────────────────────────────────────

def format_status(state, done: int, total: int, cargo=None, target=None) -> str:
    return f"STATUS {_fmt(state)} {done}/{total} {_fmt(cargo or 'NONE')} {_fmt(target or 'NONE')}"


def format_event(kind: str, detail: str = "") -> str:
    return f"EVENT {kind} {detail}".rstrip()


def format_error(code: str, detail: str = "") -> str:
    return f"ERROR {code} {detail}".rstrip()


def format_ack(cmd: Command) -> str:
    return f"ACK {cmd.key}"


def format_nak(cmd_key: str, reason: str) -> str:
    return f"NAK {cmd_key} {reason}"


def format_pong() -> str:
    return "PONG"


def format_dev(kind: str, *values) -> str:
    """DEV RANGE 0.234 / DEV LINE -0.12 NONE [STOP_LINE [88.5]] / DEV SYS 52.1 37 41 / DEV NODES a,b,c"""
    if kind == "NODES":
        return "DEV NODES " + ",".join(values[0]) if values and values[0] else "DEV NODES"
    return " ".join(["DEV", kind, *(_fmt(v) for v in values)])


def parse_reply(line: str) -> dict:
    """노트북 쪽: 차량 메시지 → dict. 알 수 없으면 ValueError."""
    parts = line.strip().split(" ")
    head, rest = parts[0], parts[1:]
    try:
        if head == "STATUS":
            done, total = rest[1].split("/")
            return {"type": "STATUS", "state": rest[0], "done": int(done), "total": int(total),
                    "cargo": rest[2], "target": rest[3]}
        if head == "EVENT":
            return {"type": "EVENT", "event": rest[0], "detail": " ".join(rest[1:])}
        if head == "ERROR":
            return {"type": "ERROR", "code": rest[0], "detail": " ".join(rest[1:])}
        if head == "ACK":
            return {"type": "ACK", "command": " ".join(rest)}
        if head == "NAK":
            key = command_key(" ".join(rest))
            n = len(key.split()) if key else 1
            return {"type": "NAK", "command": " ".join(rest[:n]), "reason": " ".join(rest[n:])}
        if head == "PONG":
            return {"type": "PONG"}
        if head == "DEV":
            kind = rest[0]
            if kind == "RANGE":
                return {"type": "DEV", "kind": kind, "distance": float(rest[1])}
            if kind == "LINE":                  # DEV LINE <offset> <event> [<raw_event> [<angle|->]]
                angle = rest[4] if len(rest) > 4 else "-"
                return {"type": "DEV", "kind": kind, "offset": float(rest[1]), "event": rest[2],
                        "raw_event": rest[3] if len(rest) > 3 else rest[2],
                        "angle": None if angle == "-" else float(angle)}
            if kind == "SYS":
                return {"type": "DEV", "kind": kind, "temp": float(rest[1]),
                        "cpu": float(rest[2]), "mem": float(rest[3])}
            if kind == "DRIVE":                 # DEV DRIVE <RUNNING|DONE|REJECTED> [<결과·이유>]
                return {"type": "DEV", "kind": kind, "state": rest[1], "result": " ".join(rest[2:])}
            if kind == "NODES":
                return {"type": "DEV", "kind": kind, "nodes": rest[1].split(",") if len(rest) > 1 else []}
    except (IndexError, ValueError) as e:
        raise ValueError(f"형식 오류: {line!r} ({e})") from None
    raise ValueError(f"알 수 없는 메시지: {line!r}")

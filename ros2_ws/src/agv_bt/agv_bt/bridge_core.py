"""bt_bridge_node의 판단 로직 (② 계층, ROS 없음) — 명령 검증·허용 상태·하트비트·DEV 전달.

출력은 io 객체의 메서드로 내보낸다 (노드는 이를 ROS 토픽 발행으로 연결, 시험은 가짜 io로 확인).
  io.send(line) / io.estop(bool) / io.cmd_vel(v, w) / io.led(state) / io.pose(name)
  io.servo(pan, tilt) / io.wheels([l1..l4]) / io.speed(v) / io.drive("FOLLOW"|"STOP") / io.tune(key, v)
  io.node_names() -> list / io.calibrate("AUTO"|"RESET")
  io.mission(cmd)

standalone=True : mission_node가 없는 단위 개발 단계. 상태(IDLE·MANUAL·ESTOP)를 브리지가 직접 관리한다.
standalone=False: SETUP·START·STOP·RESUME·MANUAL·MOVE는 mission_node로 넘기고 상태·매핑은 /mission/status를 따른다.
  START는 매핑 4개가 없으면 여기서 거부(NAK). mission_node가 없으면 상태가 바뀌지 않으므로 standalone으로 쓴다.
  DEV DRIVE ARRIVE(도착 시험)는 mission_node TEST_ARRIVE로, DEV DRIVE STOP은 주행 정지 + mission_node CANCEL (2026-11-06).
"""
from agv_protocol.messages import (
    WATCH_KEYS, Command, check_allowed, command_key, format_ack, format_dev, format_error, format_event,
    format_map, format_nak, format_pong, format_status, parse_command,
)


class BridgeCore:
    def __init__(self, io, standalone: bool = True):
        self.io = io
        self.standalone = standalone
        self.state = "IDLE"
        self.mode, self.done, self.total, self.cargo, self.load, self.target = "NONE", 0, 0, "NONE", 0, "NONE"
        self.track_map: dict = {}            # 지점 → 마커 색 (mission_node가 알려 줌)
        self.watches: set[str] = set()
        self.link_lost = False
        self.driving = False                 # DEV DRIVE FOLLOW로 drive_node가 주행 중 (노드가 set_driving으로 알림)

    # ── 상태 ─────────────────────────────────────────────

    def set_driving(self, on: bool) -> None:
        self.driving = on

    def status_line(self) -> str:
        return format_status(self.state, self.mode, self.done, self.total, self.cargo, self.load, self.target)

    def _set_state(self, state):
        if state != self.state:
            self.state = state
            self.io.send(self.status_line())

    def on_mission_status(self, state, mode, done, total, cargo, load, target, track_map: dict):
        """standalone=False일 때 /mission/status 수신. 매핑이 바뀌면 MAP도 보낸다."""
        before = self.status_line()
        self.mode, self.done, self.total, self.cargo, self.load, self.target = mode, done, total, cargo, load, target
        if track_map != self.track_map:
            self.track_map = dict(track_map)
            self.io.send(format_map(self.track_map))
        if state != self.state:
            self._set_state(state)
        elif self.status_line() != before:
            self.io.send(self.status_line())         # 개수(7세그먼트)처럼 상태 안에서 바뀌는 값도 바로 보냄

    # ── 연결·하트비트 ─────────────────────────────────────

    def on_connect(self):
        self.io.send(self.status_line())
        self.io.send(format_map(self.track_map))
        if self.link_lost:
            self.io.send(format_error("E_LINK", "연결 끊김 후 재연결 - RESUME 필요"))

    def on_heartbeat(self, ever_connected: bool, alive: bool):
        """주기적으로 호출. 첫 연결 전에는 감시하지 않는다 (functional_design F10)."""
        if not ever_connected or alive or self.link_lost:
            return
        self.link_lost = True
        self.watches.clear()
        self._emergency_stop()

    def _emergency_stop(self):
        self.io.estop(True)
        if self.standalone:
            self._set_state("ESTOP")
        else:
            self.io.mission(parse_command("STOP"))

    # ── 명령 ─────────────────────────────────────────────

    def on_line(self, line: str):
        try:
            cmd = parse_command(line)
        except ValueError as e:
            key = command_key(line) or "?"
            self.io.send(format_nak(key, str(e).removeprefix(f"{key}: ")))
            return
        if cmd.kind == "PING":
            self.io.send(format_pong())
            return
        if cmd.kind == "STOP":                      # 상태와 무관하게 즉시 (최우선)
            self.io.estop(True)
        reason = check_allowed(cmd, self.state)
        if not reason and self.driving and cmd.key in ("MANUAL ON", "DEV WHEEL", "DEV DRIVE")                 and cmd.args[-1:] != ["STOP"]:
            reason = "라인 주행 중 — DEV DRIVE STOP 먼저"           # /cmd_vel·바퀴 명령이 겹치지 않게
        if not reason and self.standalone and cmd.key == "DEV DRIVE" and cmd.args[1:] == ["ARRIVE"]:
            reason = "mission_node 없음 — agv.launch.py로 실행"
        if reason:
            self.io.send(format_nak(cmd.key, reason))
            return
        if cmd.kind == "DEV":
            self.io.send(format_ack(cmd))
            self._dev(cmd.args[0], cmd.args[1:])
        elif self.standalone:
            self._standalone(cmd)
        else:
            if cmd.kind == "START" and not all(self.track_map.get(t) for t in ("WORK", "DROP_A", "DROP_B", "PARK")):
                self.io.send(format_nak("START", "초기 설정 필요"))
                return
            if cmd.kind == "RESUME":
                self.link_lost = False
            self.io.send(format_ack(cmd))
            self.io.mission(cmd)

    def _standalone(self, cmd):
        a = cmd.args
        if cmd.kind in ("START", "SETUP"):
            self.io.send(format_nak(cmd.kind, "mission_node 없음 (단위 개발 단계)"))
            return
        self.io.send(format_ack(cmd))
        if cmd.kind == "STOP":
            self._set_state("ESTOP")
        elif cmd.kind == "RESUME":
            self.link_lost = False
            self.io.estop(False)
            self._set_state("IDLE")
            self.io.send(format_event("RESUMED"))
        elif cmd.key == "MANUAL ON":
            self._set_state("MANUAL")
        elif cmd.key == "MANUAL OFF":
            self.io.cmd_vel(0.0, 0.0)
            self._set_state("IDLE")
        elif cmd.kind == "MOVE":
            self.io.cmd_vel(float(a[0]), float(a[1]))

    def _dev(self, sub, rest):
        if sub == "LED":
            self.io.led(rest[0])
        elif sub == "POSE":
            self.io.pose(rest[0])
        elif sub == "SERVO":
            self.io.servo(float(rest[0]), float(rest[1]))
        elif sub == "WHEEL":
            self.io.wheels([float(v) for v in rest])
        elif sub == "SPEED":
            self.io.speed(float(rest[0]))
        elif sub == "DRIVE" and rest[0] == "ARRIVE":
            self.io.mission(Command("TEST_ARRIVE"))
        elif sub == "DRIVE":
            self.io.drive(rest[0])
            if rest[0] == "STOP" and not self.standalone:
                self.io.mission(Command("CANCEL"))          # 도착 시험·초기 설정·작업 중이었으면 mission도 멈춤
        elif sub == "TUNE":
            self.io.tune(rest[0], float(rest[1]))
        elif sub == "TARGET":
            self.io.target(rest[0])
        elif sub == "WATCH":
            keys = WATCH_KEYS if rest[0] == "ALL" else (rest[0],)
            for k in keys:
                (self.watches.add if rest[1] == "ON" else self.watches.discard)(k)
        elif sub == "NODES":
            self.io.send(format_dev("NODES", sorted(self.io.node_names())))
        elif sub == "CAL":
            self.io.calibrate(rest[0])

    def on_calibration(self, text: str):
        """pantilt_node /pantilt/calibration → 앱 (DEV CAL RUNNING 66.0 87.5 / DONE 66.0 / FAIL 이유 …)."""
        self.io.send(f"DEV CAL {text}".rstrip())

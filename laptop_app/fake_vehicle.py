"""가짜 차량 — Pi 없이 PC에서 노트북 앱을 시험하기 위한 시뮬레이터.

실제 차량과 같은 RfcommServer(agv_bt)·메시지 규칙(agv_protocol)을 TCP로 띄우고,
작업 사이클 상태 변화, 개발자 명령(DEV), 측정값, 영상(채널 2)을 흉내 낸다.

    python fake_vehicle.py                 # 127.0.0.1:9000 (영상 9001)
    python main.py --fake                  # 앱이 가짜 차량을 함께 띄움
"""
import argparse
import io
import math
import random
import threading
import time
from collections import deque

import _paths  # noqa: F401  (agv_protocol, agv_bt 경로)
from agv_bt.rfcomm_server import RfcommServer
from agv_protocol.messages import (
    WATCH_KEYS, check_allowed, command_key, format_ack, format_dev, format_error, format_event,
    format_nak, format_pong, format_status, parse_command,
)
from agv_protocol.types import CARGO_LED, CARGO_TARGET, Cargo, Target

POSES = {"LINE": (65, 150), "JUNCTION": (65, 165), "SIGN": (65, 180)}
MISSION_STATES = {"UTURN", "NAVIGATE", "AT_JUNCTION", "CHECK_SIGN", "APPROACH", "LOAD", "UNLOAD", "PARKED"}
FAKE_NODES = ["camera", "line_node", "marker_node", "classifier_node", "drive_node", "mission_node",
              "motor_node", "pantilt_node", "ultrasonic_node", "led_node", "bt_bridge_node"]


class FakeVehicle:
    def __init__(self, port: int = 9000, fps: float = 5.0):
        self.server = RfcommServer(on_line=self.on_line, on_connect=self.on_connect,
                                   on_disconnect=self.on_disconnect,
                                   transport="tcp", tcp_host="127.0.0.1", tcp_port=port)
        self.fps = fps
        self.lock = threading.RLock()
        self.state, self.done, self.total = "IDLE", 0, 0
        self.cargo, self.target = Cargo.NONE, None
        self.plan: deque = deque()
        self.step_end = 0.0
        self.resume = None                  # (상태, 남은 시간) — ESTOP 해제용
        self.led, self.pan, self.tilt = "OFF", *POSES["LINE"]
        self.wheels, self.last_wheel = [0.0] * 4, 0.0
        self.move, self.last_move = (0.0, 0.0), 0.0
        self.watches: set[str] = set()
        self.link_lost = False
        self.cycle = 0
        self.offset = 0.0

    # ── 로그·전송 ─────────────────────────────────────────

    def log(self, text):
        print(time.strftime("%H:%M:%S"), text, flush=True)

    def send(self, line):
        self.server.send_line(line)

    def send_status(self):
        self.send(format_status(self.state, self.done, self.total, self.cargo, self.target))

    def set_state(self, state):
        if state != self.state:
            self.log(f"상태 {self.state} → {state}")
            self.state = state
            self.send_status()

    # ── 연결 ─────────────────────────────────────────────

    def on_connect(self):
        self.log("노트북 연결됨")
        with self.lock:
            self.send_status()
            if self.link_lost:
                self.send(format_error("E_LINK", "연결 끊김 후 재연결 - RESUME 필요"))

    def on_disconnect(self):
        self.log("노트북 연결 끊김")

    def _estop(self, reason):
        if self.state == "ESTOP":
            return
        remaining = max(0.0, self.step_end - time.monotonic())
        self.resume = (self.state, remaining)
        self.wheels, self.move = [0.0] * 4, (0.0, 0.0)
        self.set_state("ESTOP")
        self.log(f"비상 정지: {reason}")

    # ── 명령 처리 ─────────────────────────────────────────

    def on_line(self, line):
        if line != "PING":
            self.log(f"수신 {line}")
        try:
            cmd = parse_command(line)
        except ValueError as e:
            key = command_key(line) or "?"
            self.send(format_nak(key, str(e).removeprefix(f"{key}: ")))
            return
        if cmd.kind == "PING":
            self.send(format_pong())
            return
        with self.lock:
            reason = check_allowed(cmd, self.state)
            if reason:
                self.send(format_nak(cmd.key, reason))
                return
            self.send(format_ack(cmd))
            self.handle(cmd)

    def handle(self, cmd):
        now = time.monotonic()
        a = cmd.args
        if cmd.kind == "START":
            self.total, self.done, self.cycle = int(a[0]), 0, 0
            self.plan = deque([("UTURN", 1.0, None), ("NAVIGATE", 2.0, None)])
            self.target = Target.WORK
            self.send(format_event("STARTED", f"{self.total}회"))
            self.plan.extend(self._cycle_steps())
            self._next_step(now)
        elif cmd.kind == "STOP":
            self._estop("STOP 명령")
        elif cmd.kind == "RESUME":
            state, remaining = self.resume or ("IDLE", 0.0)
            self.resume, self.link_lost = None, False
            self.step_end = now + remaining
            self.set_state(state if state not in ("ESTOP", "ERROR") else "IDLE")
            self.send(format_event("RESUMED"))
        elif cmd.key == "MANUAL ON":
            self.plan.clear()
            self.resume = None
            self.set_state("MANUAL")
        elif cmd.key == "MANUAL OFF":
            self.move = (0.0, 0.0)
            self.set_state("IDLE")
        elif cmd.kind == "MOVE":
            self.move, self.last_move = (float(a[0]), float(a[1])), now
        elif cmd.kind == "DEV":
            self.handle_dev(a[0], a[1:], now)

    def handle_dev(self, sub, rest, now):
        if sub == "LED":
            self.led = rest[0]
        elif sub == "POSE":
            self.pan, self.tilt = POSES[rest[0]]
        elif sub == "SERVO":
            self.pan, self.tilt = float(rest[0]), float(rest[1])
        elif sub == "WHEEL":
            self.wheels, self.last_wheel = [float(v) for v in rest], now
        elif sub == "WATCH":
            keys = WATCH_KEYS if rest[0] == "ALL" else (rest[0],)
            for k in keys:
                (self.watches.add if rest[1] == "ON" else self.watches.discard)(k)
        elif sub == "NODES":
            self.send(format_dev("NODES", FAKE_NODES))

    # ── 작업 사이클 흉내 ──────────────────────────────────

    def _cycle_steps(self):
        cargo = Cargo.CIRCLE if self.cycle % 2 == 0 else Cargo.SQUARE
        self.cycle += 1
        return [("CHECK_SIGN", 1.0, None), ("APPROACH", 1.5, None), ("LOAD", 3.0, ("load", cargo)),
                ("UTURN", 1.0, None), ("NAVIGATE", 1.5, None), ("AT_JUNCTION", 0.8, None),
                ("NAVIGATE", 1.5, None), ("AT_JUNCTION", 0.8, None), ("NAVIGATE", 1.5, None),
                ("CHECK_SIGN", 1.0, None), ("UNLOAD", 1.0, ("unload", None)), ("UTURN", 1.0, None)]

    def _run_action(self, action):
        kind, value = action
        if kind == "load":
            self.cargo, self.target, self.led = value, CARGO_TARGET[value], CARGO_LED[value].value
            self.send(format_event("LOADED", value.value))
        elif kind == "unload":
            self.done += 1
            self.send(format_event("UNLOADED", f"{self.cargo.value} {self.done}/{self.total}"))
            self.cargo, self.led = Cargo.NONE, "OFF"
            back = [("NAVIGATE", 1.5, None), ("AT_JUNCTION", 0.8, None), ("NAVIGATE", 1.5, None)]
            if self.done < self.total:
                self.target = Target.WORK
                self.plan.extend(back + [("AT_JUNCTION", 0.8, None), ("NAVIGATE", 1.0, None)] + self._cycle_steps())
            else:
                self.target = Target.PARK
                self.plan.extend(back + [("AT_JUNCTION", 0.8, None), ("NAVIGATE", 1.0, None),
                                         ("CHECK_SIGN", 1.0, None), ("PARKED", 2.0, ("park", None))])
        elif kind == "park":
            self.send(format_event("PARKED"))
            self.target = None

    def _next_step(self, now):
        if not self.plan:
            self.set_state("IDLE")
            return
        state, duration, action = self.plan.popleft()
        self.current_action = action
        self.step_end = now + duration
        self.set_state(state)

    def _advance(self, now):
        if self.state in MISSION_STATES and now >= self.step_end:
            action = getattr(self, "current_action", None)
            self.current_action = None
            if action:
                self._run_action(action)
            self._next_step(now)

    # ── 주기 작업 ─────────────────────────────────────────

    def tick(self, now, counters):
        with self.lock:
            srv = self.server
            if srv.ever_connected and not srv.is_alive(now) and self.state != "ESTOP":
                self.link_lost = True
                self.watches.clear()
                self._estop("하트비트 끊김")
            if now - self.last_wheel > 0.5:
                self.wheels = [0.0] * 4
            if now - self.last_move > 0.5:
                self.move = (0.0, 0.0)
            self._advance(now)
            if self.state == "MANUAL":
                self.offset = max(-1.0, min(1.0, self.offset * 0.97 - self.move[1] * 0.05))
            elif self.state not in ("ESTOP", "IDLE", "PARKED"):
                self.offset = 0.35 * math.sin(now * 1.3)
            counters["n"] += 1
            n = counters["n"]
            if n % 10 == 0:                                   # 2Hz
                self.send_status()
            if n % 4 == 0:                                    # 5Hz
                if "RANGE" in self.watches:
                    self.send(format_dev("RANGE", round(self._range(now), 3)))
                if "LINE" in self.watches:
                    self.send(format_dev("LINE", round(self.offset, 2), self._line_event()))
            if n % 20 == 0 and "SYS" in self.watches:         # 1Hz
                self.send(format_dev("SYS", round(48 + 4 * math.sin(now / 7), 1),
                                     float(random.randint(20, 45)), 38.0))

    def _range(self, now):
        if self.state == "APPROACH":
            return 0.10 + 0.25 * max(0.0, self.step_end - now) / 1.5
        if self.state == "LOAD":
            return 0.10 + random.uniform(-0.005, 0.005)
        return 1.2 + 0.05 * math.sin(now) + random.uniform(-0.01, 0.01)

    def _line_event(self):
        return {"AT_JUNCTION": "JUNCTION_T", "CHECK_SIGN": "STOP_LINE"}.get(self.state, "NONE")

    # ── 영상 ─────────────────────────────────────────────

    def render(self, now):
        from PIL import Image, ImageDraw
        img = Image.new("RGB", (320, 240), (225, 225, 220))
        d = ImageDraw.Draw(img)
        with self.lock:
            state, led, offset = self.state, self.led, self.offset
            pan, tilt, wheels = self.pan, self.tilt, list(self.wheels)
        if state == "CHECK_SIGN" or tilt > 80:                # 카메라를 올린 장면: 표지판
            d.rectangle((110, 60, 210, 160), fill="white", outline="black", width=4)
            letter = {Target.WORK: "W", Target.DROP_A: "A", Target.DROP_B: "B", Target.PARK: "P"}.get(self.target, "?")
            d.text((152, 100), letter, fill="black")
        else:                                                 # 아래를 본 장면: 검은 라인
            cx = 160 + offset * 110
            d.polygon([(cx - 18, 240), (cx + 18, 240), (160 + offset * 40 + 8, 90), (160 + offset * 40 - 8, 90)],
                      fill="black")
            if state == "AT_JUNCTION":
                d.rectangle((0, 150, 320, 168), fill="black")
            if state in ("CHECK_SIGN", "APPROACH"):
                d.rectangle((90, 200, 230, 212), fill="black")
        d.rectangle((0, 0, 320, 18), fill=(40, 40, 40))
        d.text((4, 3), f"FAKE  {state}  pan {pan:.0f} tilt {tilt:.0f}", fill="white")
        color = {"RED": "red", "GREEN": "lime", "ERROR": "orange"}.get(led, (90, 90, 90))
        d.ellipse((298, 2, 314, 16), fill=color)
        if any(wheels):
            d.text((4, 224), "WHEEL " + " ".join(f"{w:+.1f}" for w in wheels), fill="blue")
        d.text((250, 224), time.strftime("%H:%M:%S"), fill=(60, 60, 60))
        buf = io.BytesIO()
        img.save(buf, "JPEG", quality=50)
        return buf.getvalue()

    # ── 실행 ─────────────────────────────────────────────

    def run(self):
        self.server.start()
        self.log(f"가짜 차량 대기 중: {self.server.where} (Ctrl+C 종료)")
        counters = {"n": 0}
        next_frame = 0.0
        try:
            while True:
                now = time.monotonic()
                self.tick(now, counters)
                if now >= next_frame and self.server._video is not None:
                    next_frame = now + 1.0 / self.fps
                    self.server.send_frame(self.render(now))
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass
        finally:
            self.server.stop()


def main():
    parser = argparse.ArgumentParser(description="가짜 차량 (노트북 앱 시험용)")
    parser.add_argument("--port", type=int, default=9000, help="제어 포트 (영상은 +1)")
    parser.add_argument("--fps", type=float, default=5.0)
    args = parser.parse_args()
    FakeVehicle(args.port, args.fps).run()


if __name__ == "__main__":
    main()

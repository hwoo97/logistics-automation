"""가짜 차량 — Pi 없이 PC에서 노트북 앱을 시험하기 위한 시뮬레이터.

실제 차량과 같은 RfcommServer(agv_bt)·메시지 규칙(agv_protocol)을 TCP로 띄우고, 실제 상태 머신(agv_control.mission)을
가짜 트랙에서 돌린다 — 초기 설정(트랙 매핑)·작업(모양별 상차·하차), 개발자 명령(DEV), 측정값, 영상(채널 2)을 흉내 낸다.

    python fake_vehicle.py                 # 127.0.0.1:9000 (영상 9001)
    python main.py --fake                  # 앱이 가짜 차량을 함께 띄움
"""
import argparse
import io
import math
import random
import threading
import time

import _paths  # noqa: F401  (agv_protocol, agv_bt, agv_control 경로)
from agv_bt.rfcomm_server import RfcommServer
from agv_control.mission import Event, MissionConfig, MissionStateMachine
from agv_protocol.messages import (
    WATCH_KEYS, check_allowed, command_key, format_ack, format_dev, format_error, format_event, format_map,
    format_nak, format_pong, format_status, parse_command,
)

POSES = {"LINE": (70, 130), "JUNCTION": (65, 165), "SIGN": (65, 165)}
FAKE_NODES = ["camera", "line_node", "marker_node", "classifier_node", "drive_node", "mission_node",
              "motor_node", "pantilt_node", "ultrasonic_node", "led_node", "segment_node", "bt_bridge_node"]
FAKE_TRACK = {"red": "DROP_A", "green": "DROP_B", "blue": "WORK", "yellow": "PARK"}   # 가짜 트랙의 실제 배치
SIGN_LABEL = {"WORK": "sign_work", "DROP_A": "sign_drop_a", "DROP_B": "sign_drop_b", "PARK": "sign_park"}
LETTER = {"WORK": "W", "DROP_A": "A", "DROP_B": "B", "PARK": "P"}
DRIVE_TIME = {"UTURN": 1.5, "FOLLOW_TO_EVENT": 3.0}       # 가짜 동작에 걸리는 시간(초)


class FakeVehicle:
    def __init__(self, port: int = 9000, fps: float = 5.0):
        self.server = RfcommServer(on_line=self.on_line, on_connect=self.on_connect,
                                   on_disconnect=self.on_disconnect,
                                   transport="tcp", tcp_host="127.0.0.1", tcp_port=port)
        self.fps = fps
        self.lock = threading.RLock()
        self.sm = MissionStateMachine(MissionConfig())
        self.pending: list = []             # (시각, Event, 주행 결과면 True) — 가짜 동작이 끝나면 넣을 이벤트
        self.at, self.color = "PARK", "NONE"   # 지금 있는 지점, 따라가는 색
        self.digit = "0"
        self.sent_status, self.sent_map = None, None
        self.led, self.pan, self.tilt = "OFF", *POSES["LINE"]
        self.wheels, self.last_wheel = [0.0] * 4, 0.0
        self.move, self.last_move = (0.0, 0.0), 0.0
        self.watches: set[str] = set()
        self.link_lost = False
        self.offset = 0.0
        self.drive_speed = 0.6                     # DEV SPEED (drive_node 기본 속도 흉내)
        self.drive_end = None                      # DEV DRIVE FOLLOW — 이 시각에 정지선 도착 흉내
        self.drive_uturn_end = None                # DEV DRIVE UTURN — 이 시각에 유턴 완료 흉내

    @property
    def state(self) -> str:
        return self.sm.state.value

    # ── 로그·전송 ─────────────────────────────────────────

    def log(self, text):
        print(time.strftime("%H:%M:%S"), text, flush=True)

    def send(self, line):
        self.server.send_line(line)

    def send_status(self, force=False):
        s = self.sm.status()
        line = format_status(s["state"], s["mode"], s["done"], s["total"], s["cargo"], s["load"], s["target"])
        if s["map"] != self.sent_map or force:
            self.sent_map = dict(s["map"])
            self.send(format_map(s["map"]))
        if line != self.sent_status or force:
            if self.sent_status and line.split()[1] != self.sent_status.split()[1]:
                self.log(f"상태 {self.sent_status.split()[1]} → {s['state']}")
            self.sent_status = line
            self.send(line)

    # ── 연결 ─────────────────────────────────────────────

    def on_connect(self):
        self.log("노트북 연결됨")
        with self.lock:
            self.send_status(force=True)
            if self.link_lost:
                self.send(format_error("E_LINK", "연결 끊김 후 재연결 - RESUME 필요"))

    def on_disconnect(self):
        self.log("노트북 연결 끊김")

    # ── 상태 머신 ─────────────────────────────────────────

    def feed(self, kind, now=None, **data):
        data["now"] = time.monotonic() if now is None else now
        for act in self.sm.handle(Event(kind, data)):
            self.apply(act, data["now"])
        self.send_status()

    def later(self, now, delay, kind, drive=False, **data):
        self.pending.append((now + delay, Event(kind, data), drive))

    def apply(self, act, now):
        """상태 머신 동작을 가짜 트랙에서 흉내 내 결과 이벤트를 예약한다."""
        k, d = act.kind, act.data
        if k == "DRIVE":
            self.pending = [p for p in self.pending if not p[2]]
            cmd = d["command"]
            if cmd == "UTURN":
                self.later(now, DRIVE_TIME["UTURN"], "DRIVE_DONE", True, result="UTURN_DONE")
            elif cmd == "FOLLOW_TO_EVENT":
                dest = FAKE_TRACK.get(self.color)
                if dest:
                    self.at = dest
                self.later(now, DRIVE_TIME["FOLLOW_TO_EVENT"] / max(0.2, self.drive_speed) * 0.6, "DRIVE_DONE", True,
                           result="STOP_LINE" if dest else "LINE_END")
            elif cmd == "APPROACH":
                for i, dist in enumerate((0.40, 0.32, 0.25, 0.19, 0.18, 0.17, 0.17)):
                    self.later(now, 0.3 * (i + 1), "RANGE", True, distance=dist)
        elif k == "CANCEL_DRIVE":
            self.pending = [p for p in self.pending if not p[2]]
        elif k == "TARGET":
            self.color = d["color"]
        elif k == "POSE":
            self.pan, self.tilt = POSES[d["pose"]]
            self.later(now, 0.3, "POSE_READY", pose=d["pose"])
        elif k == "CLASSIFY":
            self.later(now, 1.0, "SIGN", label=SIGN_LABEL[self.at], score=0.95)
        elif k == "LED":
            self.led = d["state"]
        elif k == "DIGIT":
            self.digit = d["char"]
        elif k == "CMD_VEL":
            self.move, self.last_move = (d["linear"], d["angular"]), now
        elif k == "REPORT":
            line = format_error(d["error"], d.get("detail", "")) if "error" in d else format_event(d["event"], d.get("detail", ""))
            self.send(line)
            self.log(line)
        elif k == "ESTOP" and d["on"]:
            self.wheels, self.move = [0.0] * 4, (0.0, 0.0)

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
            if not reason and cmd.kind == "START" and not self.sm.mapped:
                reason = "초기 설정 필요"
            if reason:
                self.send(format_nak(cmd.key, reason))
                return
            self.send(format_ack(cmd))
            self.handle(cmd)

    def handle(self, cmd):
        now = time.monotonic()
        a = cmd.args
        if cmd.kind == "START":
            self.feed("START", circle=int(a[0]), square=int(a[1]))
        elif cmd.kind == "MOVE":
            self.feed("MOVE", linear=float(a[0]), angular=float(a[1]))
        elif cmd.kind == "MANUAL":
            self.feed(f"MANUAL_{a[0]}")
        elif cmd.kind == "RESUME":
            self.link_lost = False
            self.feed("RESUME")
        elif cmd.kind in ("SETUP", "STOP"):
            if cmd.kind == "STOP":
                self.pending = [p for p in self.pending if not p[2]]
                self._stop_dev_drive()
            self.feed(cmd.kind)
        elif cmd.kind == "DEV":
            self.handle_dev(a[0], a[1:], now)

    def _stop_dev_drive(self):
        if self.drive_end is not None:
            self.drive_end = None
            self.send(format_dev("DRIVE", "DONE", "E_ESTOP"))

    def handle_dev(self, sub, rest, now):
        if sub == "LED":
            self.led = rest[0]
        elif sub == "DIGIT":
            self.digit = rest[0]
        elif sub == "POSE":
            self.pan, self.tilt = POSES[rest[0]]
        elif sub == "SERVO":
            self.pan, self.tilt = float(rest[0]), float(rest[1])
        elif sub == "WHEEL":
            self.wheels, self.last_wheel = [float(v) for v in rest], now
        elif sub == "SPEED":
            self.drive_speed = float(rest[0])
        elif sub == "TUNE":
            setattr(self, "drive_" + rest[0].lower(), float(rest[1]))   # drive_turn, drive_center
        elif sub == "TARGET":
            self.drive_target = rest[0]
        elif sub == "DRIVE":
            if rest[0] == "FOLLOW":
                self.drive_end = now + 6.0 / self.drive_speed      # 빠를수록 일찍 도착
                self.send(format_dev("DRIVE", "RUNNING"))
            elif rest[0] == "UTURN":
                self.drive_uturn_end = now + 3.0
                self.send(format_dev("DRIVE", "UTURN"))
            elif rest[0] == "ARRIVE":                              # 도착 시험 — mission_node 흉내 (목적지 색 = DEV TARGET)
                target = getattr(self, "drive_target", "NONE")
                self.color = target.lower() if target != "NONE" else "NONE"
                self.feed("TEST_ARRIVE", now)
            else:
                self.drive_end = self.drive_uturn_end = None
                self.send(format_dev("DRIVE", "DONE", "STOPPED"))
                self.feed("CANCEL", now)
        elif sub == "WATCH":
            keys = WATCH_KEYS if rest[0] == "ALL" else (rest[0],)
            for k in keys:
                (self.watches.add if rest[1] == "ON" else self.watches.discard)(k)
        elif sub == "NODES":
            self.send(format_dev("NODES", FAKE_NODES))
        elif sub == "CAL":                                       # 정면 자동 맞춤 흉내 — 70° → 66°
            if rest[0] == "SIGN":                                 # SIGN 틸트 저장 흉내
                self.pan, self.tilt = self.pan, float(rest[1])
                self.send(format_dev("CAL", "SIGN", int(float(rest[1]))))
            elif rest[0] == "AUTO":
                for pan, angle in ((70.0, 84.0), (73.0, 79.5), (66.5, 89.2), (66.0, 90.0)):
                    self.send(format_dev("CAL", "RUNNING", pan, angle))
                self.pan = 66.0
                self.send(format_dev("CAL", "DONE", 66.0))
            else:
                self.pan = 70.0
                self.send(format_dev("CAL", "RESET", 70.0))

    # ── 주기 작업 ─────────────────────────────────────────

    def tick(self, now, counters):
        with self.lock:
            srv = self.server
            if srv.ever_connected and not srv.is_alive(now) and self.state != "ESTOP" and not self.link_lost:
                self.link_lost = True
                self.watches.clear()
                self.log("비상 정지: 하트비트 끊김")
                self.pending = [p for p in self.pending if not p[2]]
                self.feed("STOP", now)
            if now - self.last_wheel > 0.5:
                self.wheels = [0.0] * 4
            if now - self.last_move > 0.5:
                self.move = (0.0, 0.0)
            due = [p for p in self.pending if p[0] <= now]
            self.pending = [p for p in self.pending if p[0] > now]
            for _, ev, _ in sorted(due, key=lambda p: p[0]):
                self.feed(ev.kind, now, **{k: v for k, v in ev.data.items() if k != "now"})
            self.feed("TICK", now)
            if self.drive_end is not None and now >= self.drive_end:
                self.drive_end = None
                for n in (1, 2, 3):                       # 라인 끝 재확인 흉내
                    self.send(format_dev("DRIVE", "RECHECK", f"{n}/3"))
                self.send(format_dev("DRIVE", "DONE", "LINE_END"))
            if self.drive_uturn_end is not None and now >= self.drive_uturn_end:
                self.drive_uturn_end = None
                self.send(format_dev("DRIVE", "DONE", "UTURN_DONE"))
            if self.state == "MANUAL":
                self.offset = max(-1.0, min(1.0, self.offset * 0.97 - self.move[1] * 0.05))
            elif self.state not in ("ESTOP", "IDLE", "PARKED"):
                self.offset = 0.35 * math.sin(now * 1.3)
            counters["n"] += 1
            n = counters["n"]
            if n % 10 == 0:                                   # 2Hz
                self.send_status(force=True)
            if n % 4 == 0:                                    # 5Hz
                if "RANGE" in self.watches:
                    self.send(format_dev("RANGE", round(self._range(now), 3)))
                if "LINE" in self.watches:
                    ev = self._line_event()
                    angle = round(90 - 25 * self.offset + 6 * math.sin(now * 0.7), 1)   # 각도 흉내
                    self.send(format_dev("LINE", round(self.offset, 2), ev, ev, angle))
            if n % 20 == 0 and "SYS" in self.watches:         # 1Hz
                self.send(format_dev("SYS", round(48 + 4 * math.sin(now / 7), 1),
                                     float(random.randint(20, 45)), 38.0))

    def _range(self, now):
        if self.state in ("IDLE", "PARKED", "APPROACH", "CHECK_SIGN", "LOAD", "UNLOAD"):   # 대기장소 = 표지판 앞
            return 0.29 + random.uniform(-0.005, 0.005)      # 표지판 앞 약 30cm
        return 1.2 + 0.05 * math.sin(now) + random.uniform(-0.01, 0.01)

    def _line_event(self):
        return {"APPROACH": "STOP_LINE"}.get(self.state, "NONE")

    # ── 영상 ─────────────────────────────────────────────

    def render(self, now):
        from PIL import Image, ImageDraw
        img = Image.new("RGB", (320, 240), (225, 225, 220))
        d = ImageDraw.Draw(img)
        with self.lock:
            state, led, offset, digit = self.state, self.led, self.offset, self.digit
            pan, tilt, wheels, at = self.pan, self.tilt, list(self.wheels), self.at
        sign_tilt = (POSES["JUNCTION"][1] + POSES["SIGN"][1]) / 2
        if tilt >= sign_tilt:                                 # 카메라를 올린 장면: 표지판 (틸트는 클수록 위)
            d.rectangle((110, 60, 210, 160), fill="white", outline="black", width=4)
            d.text((152, 100), LETTER.get(at, "?"), fill="black")
        else:                                                 # 아래를 본 장면: 검은 라인
            cx = 160 + offset * 110
            d.polygon([(cx - 18, 240), (cx + 18, 240), (160 + offset * 40 + 8, 90), (160 + offset * 40 - 8, 90)],
                      fill="black")
            if state == "APPROACH":
                d.rectangle((90, 200, 230, 212), fill="black")
        d.rectangle((0, 0, 320, 18), fill=(40, 40, 40))
        d.text((4, 3), f"FAKE  {state}  pan {pan:.0f} tilt {tilt:.0f}  spd {self.drive_speed:.2f}", fill="white")
        color = {"RED": "red", "GREEN": "lime", "ERROR": "orange"}.get(led, (90, 90, 90))
        d.ellipse((298, 2, 314, 16), fill=color)
        d.rectangle((272, 1, 292, 17), fill="black")          # 7세그먼트 흉내
        d.text((279, 3), digit, fill="red")
        if any(wheels):
            d.text((4, 224), "WHEEL " + " ".join(f"{w:+.1f}" for w in wheels), fill="blue")
        d.text((250, 224), time.strftime("%H:%M:%S"), fill=(60, 60, 60))
        buf = io.BytesIO()
        img.save(buf, "JPEG", quality=50)
        return buf.getvalue()

    # ── 실행 ─────────────────────────────────────────────

    def run(self):
        self.server.start()
        self.log(f"가짜 차량 대기 중: {self.server.where} (Ctrl+C 종료) — 가짜 트랙 {FAKE_TRACK}")
        counters = {"n": 0}
        next_frame = 0.0
        try:
            while True:
                now = time.monotonic()
                self.tick(now, counters)
                if now >= next_frame and self.server.video_connected:
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

"""메인 창 — 연결 막대(공통) + 사용자 탭 / 개발자 탭.

통신 스레드의 콜백은 큐에 넣고, 메인 스레드가 30ms마다 꺼내 화면을 갱신한다 (tkinter 규칙).
"""
import json
import os
import queue
import threading
import tkinter as tk
from tkinter import ttk

import _paths  # noqa: F401
from agv_protocol.messages import parse_reply
from bt_client import BtClient
from ui.dev_tab import DevTab
from ui.user_tab import UserTab
from video_receiver import VideoReceiver

CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config.json")
TRANSPORTS = ("블루투스 (Pi)", "가짜 차량 (PC)")
TEXT_INPUTS = ("Entry", "TEntry", "Spinbox", "TSpinbox", "Text")


def load_config() -> dict:
    try:
        with open(CONFIG, encoding="utf-8") as fp:
            return json.load(fp)
    except (OSError, ValueError):
        return {}


def save_config(cfg: dict) -> None:
    try:
        with open(CONFIG, "w", encoding="utf-8") as fp:
            json.dump(cfg, fp, ensure_ascii=False, indent=2)
    except OSError:
        pass


class MainWindow:
    def __init__(self, root: tk.Tk, fake: bool = False, addr: str | None = None, fake_port: int = 9000):
        self.root = root
        self.fake_port = fake_port
        self.cfg = load_config()
        self.q: queue.Queue = queue.Queue()
        self.client = BtClient(on_line=lambda d, t: self.q.put(("line", d, t)),
                               on_closed=lambda r: self.q.put(("closed", r)))
        self.video = VideoReceiver()
        self.connecting = False
        self.last_state = None

        root.title("물류 AGV 제어")
        root.geometry("1180x800")
        root.minsize(1000, 700)
        self._build_bar(fake, addr or self.cfg.get("bt_address", ""))
        self.tabs = ttk.Notebook(root)
        self.tabs.pack(fill="both", expand=True, padx=6, pady=(0, 6))
        self.user = UserTab(self.tabs, self.send)
        self.dev = DevTab(self.tabs, self.send, self.send_line)
        self.dev.get_latest_frame = lambda: self.video.latest
        self.tabs.add(self.user, text="  사용자  ")
        self.tabs.add(self.dev, text="  개발자  ")
        self._apply_flip()

        root.bind_all("<KeyPress>", self._on_key)
        root.bind_all("<KeyRelease>", self._on_key_release)
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self._stats_tick = 0
        self.root.after(30, self.poll)

    # ── 연결 막대 ─────────────────────────────────────────

    def _build_bar(self, fake, addr):
        bar = ttk.Frame(self.root, padding=(6, 6))
        bar.pack(fill="x")
        ttk.Label(bar, text="연결").pack(side="left")
        self.transport = ttk.Combobox(bar, values=TRANSPORTS, state="readonly", width=14)
        self.transport.current(1 if fake else 0)
        self.transport.pack(side="left", padx=4)
        ttk.Label(bar, text="Pi 주소").pack(side="left")
        self.addr = ttk.Entry(bar, width=20)
        self.addr.insert(0, addr)
        self.addr.pack(side="left", padx=4)
        self.conn_btn = ttk.Button(bar, text="연결", command=self.toggle_connect)
        self.conn_btn.pack(side="left", padx=4)
        self.conn_lbl = ttk.Label(bar, text="연결 안 됨", foreground="gray", width=26)
        self.conn_lbl.pack(side="left", padx=6)
        self.flip_v = tk.BooleanVar(value=self.cfg.get("video_flip_v", False))
        self.flip_h = tk.BooleanVar(value=self.cfg.get("video_flip_h", False))
        ttk.Checkbutton(bar, text="영상 상하 반전", variable=self.flip_v, command=self._apply_flip).pack(side="left", padx=4)
        ttk.Checkbutton(bar, text="좌우 반전", variable=self.flip_h, command=self._apply_flip).pack(side="left", padx=4)
        self.estop_btn = tk.Button(bar, text="■ 비상 정지 (Space)", bg="#d32f2f", fg="white",
                                   activebackground="#b71c1c", font=("맑은 고딕", 13, "bold"),
                                   padx=14, command=self.estop)
        self.estop_btn.pack(side="right")

    def _apply_flip(self):
        """두 탭 영상에 반전 적용, 다음 실행을 위해 config.json에 저장."""
        v, h = self.flip_v.get(), self.flip_h.get()
        for view in (self.user.video, self.dev.video):
            view.flip_v, view.flip_h = v, h
        if (self.cfg.get("video_flip_v"), self.cfg.get("video_flip_h")) != (v, h):
            self.cfg["video_flip_v"], self.cfg["video_flip_h"] = v, h
            save_config(self.cfg)

    def _target(self) -> dict:
        if self.transport.current() == 1:
            return {"transport": "tcp", "host": "127.0.0.1", "port": self.fake_port}
        return {"transport": "rfcomm", "addr": self.addr.get().strip()}

    def toggle_connect(self):
        if self.client.connected:
            self.client.close()
            return
        if self.connecting:
            return
        target = self._target()
        if target["transport"] == "rfcomm":
            if not target["addr"]:
                self.conn_lbl.config(text="Pi 블루투스 주소를 입력", foreground="#d32f2f")
                return
            self.cfg["bt_address"] = target["addr"]
            save_config(self.cfg)
        self.connecting = True
        self._lock_inputs(True)
        self.conn_btn.config(state="disabled")
        self.conn_lbl.config(text="연결 중...", foreground="#f57c00")
        threading.Thread(target=self._connect_worker, args=(target,), daemon=True).start()

    def _lock_inputs(self, locked: bool):
        """연결 중·연결됨에는 연결 방식·주소를 바꾸지 못하게."""
        self.transport.config(state="disabled" if locked else "readonly")
        self.addr.config(state="disabled" if locked else "normal")

    def _connect_worker(self, target):
        try:
            self.client.connect(target)
        except OSError as e:
            self.q.put(("connect_failed", str(e)))
            return
        try:
            self.video.connect(target)
            video_err = None
        except OSError as e:
            video_err = str(e)
        self.q.put(("connected", video_err))

    # ── 전송 ─────────────────────────────────────────────

    def send(self, kind, *args) -> bool:
        if not self.client.connected:
            self.dev.log("SYS", f"연결 안 됨 — {kind} {' '.join(map(str, args))} 보내지 않음")
            return False
        try:
            return self.client.send(kind, *args)
        except ValueError as e:
            self.dev.log("ERR", f"형식 오류: {e}")
            self.user.add_event(f"형식 오류: {e}", error=True)
            return False

    def send_line(self, line) -> bool:
        if not self.client.connected:
            self.dev.log("SYS", "연결 안 됨")
            return False
        return self.client.send_line(line)

    def estop(self):
        self.dev.stop_wheels()
        if not self.send("STOP"):
            self.user.add_event("연결 안 됨 — 비상 정지를 보내지 못함", error=True)

    # ── 키보드 ───────────────────────────────────────────

    def _typing(self, event) -> bool:
        return event.widget.winfo_class() in TEXT_INPUTS if hasattr(event.widget, "winfo_class") else False

    def _on_key(self, event):
        if self._typing(event):
            return
        key = event.keysym.lower()
        if key == "space":
            self.estop()
        elif key in "wasd" and len(key) == 1 and self.tabs.select() == str(self.user):
            self.user.pad.press(key)

    def _on_key_release(self, event):
        key = event.keysym.lower()
        if len(key) == 1 and key in "wasd":
            self.user.pad.release(key)

    # ── 수신 처리 (메인 스레드) ────────────────────────────

    def poll(self):
        try:
            while True:
                self._handle(self.q.get_nowait())
        except queue.Empty:
            pass
        frame = self.video.take_new()
        if frame:
            view = self.user.video if self.tabs.select() == str(self.user) else self.dev.video
            view.show(frame)
        self._stats_tick += 1
        if self._stats_tick % 17 == 0:                 # 약 0.5초
            self.dev.update_video_stats(self.video.fps, self.video.kbps)
            if self.client.connected:
                rtt = f"{self.client.rtt_ms:.1f} ms" if self.client.rtt_ms is not None else "-"
                self.conn_lbl.config(text=f"연결됨  RTT {rtt}", foreground="#2e7d32")
        self.root.after(30, self.poll)

    def _handle(self, item):
        kind = item[0]
        if kind == "connected":
            self.connecting = False
            self.conn_btn.config(state="normal", text="해제")
            self.conn_lbl.config(text="연결됨", foreground="#2e7d32")
            self.dev.log("SYS", "연결됨" + (f" (영상 채널 실패: {item[1]})" if item[1] else ""))
            self.user.add_event("연결됨")
            self.dev.resend_watches()
        elif kind == "connect_failed":
            self.connecting = False
            self._lock_inputs(False)
            self.conn_btn.config(state="normal", text="연결")
            self.conn_lbl.config(text="연결 실패", foreground="#d32f2f")
            self.dev.log("ERR", f"연결 실패: {item[1]}")
            self.user.add_event(f"연결 실패: {item[1]}", error=True)
        elif kind == "closed":
            self.video.close()
            self._lock_inputs(False)
            self.conn_btn.config(state="normal", text="연결")
            self.conn_lbl.config(text="연결 안 됨", foreground="gray")
            self.dev.stop_wheels()
            self.user.set_disconnected()
            self.user.add_event(f"연결 끊김: {item[1]}", error=item[1] != "사용자 해제")
            self.dev.log("SYS", f"연결 끊김: {item[1]}")
            self.last_state = None
        elif kind == "line":
            direction, text = item[1], item[2]
            self.dev.log(direction, text)
            if direction == "RX":
                self._on_reply(text)

    def _on_reply(self, text):
        try:
            r = parse_reply(text)
        except ValueError as e:
            self.dev.log("ERR", str(e))
            return
        t = r["type"]
        if t == "STATUS":
            self.user.update_status(r)
            self.dev.update_state(r["state"])
            self.dev.video.set_overlay(r["state"])
            if r["state"] == "ESTOP" and self.last_state != "ESTOP":
                self.dev.stop_wheels()
            self.last_state = r["state"]
        elif t == "EVENT":
            self.user.add_event(f"{r['event']} {r['detail']}".strip())
        elif t == "ERROR":
            self.user.add_event(f"오류 {r['code']} {r['detail']}".strip(), error=True)
        elif t == "NAK":
            self.user.add_event(f"거부 {r['command']}: {r['reason']}", error=True)
        elif t == "DEV":
            self.dev.on_dev(r)

    def on_close(self):
        self.client.close()
        self.video.close()
        self.root.destroy()

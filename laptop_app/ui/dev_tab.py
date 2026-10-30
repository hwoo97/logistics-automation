"""개발자 탭 — 하드웨어 직접 조작(DEV), 측정값, 영상 상태·사진 저장, 메시지 콘솔."""
import os
import time
import tkinter as tk
from collections import deque
from tkinter import filedialog, ttk

import _paths  # noqa: F401
from agv_protocol.messages import MOVE_REPEAT_S, WATCH_KEYS
from ui.video_view import VideoView

CAPTURE_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "captures")
LOAD_DISTANCE, OBSTACLE_DISTANCE = 0.10, 0.15   # params.yaml 초기값 (그래프 기준선)


class DevTab(ttk.Frame):
    def __init__(self, master, send, send_line):
        super().__init__(master, padding=6)
        self.send = send                    # send(kind, *args) — 형식 검증
        self.send_line = send_line          # 콘솔 직접 입력용
        self.ranges: deque = deque(maxlen=120)
        self._wheel_job = None
        self.log_lines: list[str] = []

        top = ttk.Frame(self)
        top.pack(fill="x")
        self._build_controls(top).grid(row=0, column=0, sticky="nsw", padx=(0, 6))
        self._build_measure(top).grid(row=0, column=1, sticky="nsw", padx=(0, 6))
        self._build_video(top).grid(row=0, column=2, sticky="n")
        self._build_console(self).pack(fill="both", expand=True, pady=(6, 0))

    # ── 하드웨어 조작 ─────────────────────────────────────

    def _build_controls(self, master):
        f = ttk.Frame(master)
        led = ttk.LabelFrame(f, text="LED  (/led/state)")
        led.pack(fill="x", pady=(0, 4))
        self.led = tk.StringVar(value="OFF")
        for i, v in enumerate(("OFF", "RED", "GREEN", "ERROR")):
            ttk.Radiobutton(led, text=v, value=v, variable=self.led,
                            command=lambda: self.send("DEV", "LED", self.led.get())).grid(row=0, column=i, padx=2)

        pt = ttk.LabelFrame(f, text="팬틸트  (/pantilt/pose · 각도)")
        pt.pack(fill="x", pady=(0, 4))
        for i, p in enumerate(("LINE", "JUNCTION", "SIGN")):
            ttk.Button(pt, text=p, width=9, command=lambda p=p: self.send("DEV", "POSE", p)).grid(row=0, column=i, padx=2, pady=2)
        self.pan, self.tilt = tk.DoubleVar(value=65), tk.DoubleVar(value=150)
        for r, (name, var) in enumerate((("팬", self.pan), ("틸트", self.tilt)), start=1):
            ttk.Label(pt, text=name).grid(row=r, column=0, sticky="e")
            s = tk.Scale(pt, from_=0, to=180, orient="horizontal", variable=var, length=170, resolution=1)
            s.grid(row=r, column=1, columnspan=2, sticky="w")
            s.bind("<ButtonRelease-1>", lambda e: self._send_servo())

        wh = ttk.LabelFrame(f, text="바퀴 듀티  (/dev/wheels, -1~1)")
        wh.pack(fill="x")
        self.wheels = [tk.DoubleVar(value=0.0) for _ in range(4)]
        for i, var in enumerate(self.wheels):
            ttk.Label(wh, text=f"L{i + 1}").grid(row=i, column=0)
            tk.Scale(wh, from_=-1, to=1, resolution=0.05, orient="horizontal", variable=var,
                     length=170).grid(row=i, column=1, columnspan=2, sticky="w")
        self.wheel_on = tk.BooleanVar(value=False)
        ttk.Checkbutton(wh, text="보내기 (0.3초 반복)", variable=self.wheel_on, command=self._toggle_wheels).grid(
            row=4, column=0, columnspan=2, sticky="w")
        ttk.Button(wh, text="모두 0", command=self._zero_wheels).grid(row=4, column=2, sticky="e", padx=2)
        return f

    def _send_servo(self):
        self.send("DEV", "SERVO", float(round(self.pan.get())), float(round(self.tilt.get())))

    def _wheel_values(self):
        return [float(round(v.get(), 2)) for v in self.wheels]

    def _toggle_wheels(self):
        if self.wheel_on.get():
            self._repeat_wheels()
        else:
            if self._wheel_job:
                self.after_cancel(self._wheel_job)
                self._wheel_job = None
            self.send("DEV", "WHEEL", 0.0, 0.0, 0.0, 0.0)

    def _repeat_wheels(self):
        if not self.wheel_on.get():
            return
        if not self.send("DEV", "WHEEL", *self._wheel_values()):
            self.wheel_on.set(False)
            return
        self._wheel_job = self.after(int(MOVE_REPEAT_S * 1000), self._repeat_wheels)

    def _zero_wheels(self):
        for v in self.wheels:
            v.set(0.0)

    def stop_wheels(self):
        """연결 끊김·비상 정지 시 반복 전송 중단."""
        self.wheel_on.set(False)
        if self._wheel_job:
            self.after_cancel(self._wheel_job)
            self._wheel_job = None

    # ── 측정값 ───────────────────────────────────────────

    def _build_measure(self, master):
        f = ttk.Frame(master)
        w = ttk.LabelFrame(f, text="측정값 받기  (DEV WATCH)")
        w.pack(fill="x", pady=(0, 4))
        self.watch = {k: tk.BooleanVar(value=False) for k in WATCH_KEYS}
        for i, k in enumerate(WATCH_KEYS):
            ttk.Checkbutton(w, text=k, variable=self.watch[k],
                            command=lambda k=k: self.send("DEV", "WATCH", k, "ON" if self.watch[k].get() else "OFF")
                            ).grid(row=0, column=i, padx=4)

        rg = ttk.LabelFrame(f, text="초음파 거리 (m)")
        rg.pack(fill="x", pady=(0, 4))
        self.range_lbl = ttk.Label(rg, text="-", font=("Consolas", 14, "bold"))
        self.range_lbl.pack(anchor="w", padx=4)
        self.graph = tk.Canvas(rg, width=300, height=110, bg="white", highlightthickness=1)
        self.graph.pack(padx=4, pady=2)

        ln = ttk.LabelFrame(f, text="라인 오프셋 · 이벤트  (/line/state)")
        ln.pack(fill="x", pady=(0, 4))
        self.line_bar = tk.Canvas(ln, width=300, height=26, bg="white", highlightthickness=1)
        self.line_bar.pack(padx=4, pady=2)
        self.line_lbl = ttk.Label(ln, text="-", font=("Consolas", 11))
        self.line_lbl.pack(anchor="w", padx=4)

        sy = ttk.LabelFrame(f, text="Pi 상태 · ROS 노드")
        sy.pack(fill="x")
        self.sys_lbl = ttk.Label(sy, text="온도 - / CPU - / 메모리 -", font=("Consolas", 11))
        self.sys_lbl.grid(row=0, column=0, columnspan=2, sticky="w", padx=4)
        ttk.Button(sy, text="노드 목록", command=lambda: self.send("DEV", "NODES")).grid(row=1, column=0, sticky="nw", padx=4)
        self.nodes = tk.Listbox(sy, height=5, width=24)
        self.nodes.grid(row=1, column=1, padx=4, pady=2)
        self.state_lbl = ttk.Label(sy, text="차량 상태: -")
        self.state_lbl.grid(row=2, column=0, columnspan=2, sticky="w", padx=4)
        return f

    def _draw_graph(self):
        c, w, h, top = self.graph, 300, 110, 2.0
        c.delete("all")
        for d, color in ((LOAD_DISTANCE, "#2e7d32"), (OBSTACLE_DISTANCE, "#f57c00")):
            y = h - d / top * h
            c.create_line(0, y, w, y, fill=color, dash=(3, 2))
        c.create_text(w - 2, 2, anchor="ne", text=f"{top:.0f} m", fill="gray")
        pts = []
        for i, d in enumerate(self.ranges):
            pts += [i * w / (self.ranges.maxlen - 1), h - min(d, top) / top * h]
        if len(pts) >= 4:
            c.create_line(*pts, fill="#1976d2", width=2)

    def _draw_line(self, offset):
        c, w = self.line_bar, 300
        c.delete("all")
        c.create_line(w / 2, 0, w / 2, 26, fill="gray")
        x = w / 2 + offset * w / 2
        c.create_rectangle(x - 5, 3, x + 5, 23, fill="black")

    # ── 영상 ─────────────────────────────────────────────

    def _build_video(self, master):
        f = ttk.LabelFrame(master, text="영상 (채널 2)")
        self.video = VideoView(f, 320, 240)
        self.video.pack(padx=4, pady=4)
        self.video_lbl = ttk.Label(f, text="FPS - / - kbps", font=("Consolas", 11))
        self.video_lbl.pack(anchor="w", padx=4)
        ttk.Button(f, text="사진 저장", command=self._save_snapshot).pack(anchor="w", padx=4, pady=(0, 4))
        self.get_latest_frame = lambda: None    # MainWindow가 지정
        return f

    def _save_snapshot(self):
        jpeg = self.get_latest_frame()
        if not jpeg:
            self.log("SYS", "저장할 영상 없음")
            return
        os.makedirs(CAPTURE_DIR, exist_ok=True)
        path = os.path.join(CAPTURE_DIR, time.strftime("snap_%Y%m%d_%H%M%S.jpg"))
        with open(path, "wb") as fp:
            fp.write(jpeg)
        self.log("SYS", f"사진 저장: {path}")

    def update_video_stats(self, fps, kbps):
        self.video_lbl.config(text=f"FPS {fps:4.1f} / {kbps:6.0f} kbps")

    # ── 콘솔 ─────────────────────────────────────────────

    def _build_console(self, master):
        f = ttk.LabelFrame(master, text="메시지 콘솔")
        bar = ttk.Frame(f)
        bar.pack(fill="x")
        self.hide_noise = tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text="반복 메시지 숨기기 (PING·STATUS·측정값·MOVE·WHEEL)", variable=self.hide_noise).pack(side="left", padx=4)
        ttk.Button(bar, text="로그 저장", command=self._save_log).pack(side="right", padx=4)
        ttk.Button(bar, text="지우기", command=lambda: self.console.delete("1.0", "end")).pack(side="right")
        self.console = tk.Text(f, height=9, font=("Consolas", 10), wrap="none")
        self.console.pack(fill="both", expand=True, padx=4, pady=2)
        self.console.tag_config("TX", foreground="#1565c0")
        self.console.tag_config("RX", foreground="#2e7d32")
        self.console.tag_config("ERR", foreground="#d32f2f")
        self.console.tag_config("SYS", foreground="gray")
        row = ttk.Frame(f)
        row.pack(fill="x", padx=4, pady=(0, 4))
        self.entry = ttk.Entry(row)
        self.entry.pack(side="left", fill="x", expand=True)
        self.entry.bind("<Return>", lambda e: self._send_raw())
        ttk.Button(row, text="보내기", command=self._send_raw).pack(side="left", padx=4)
        ttk.Label(row, text="예: DEV LED RED / DEV SERVO 90 60", foreground="gray").pack(side="left")
        return f

    def _send_raw(self):
        line = self.entry.get().strip()
        if line:
            self.send_line(line)        # 형식 검증 없이 그대로 (차량의 NAK 확인용)
            self.entry.delete(0, "end")

    def log(self, direction: str, text: str):
        stamp = time.strftime("%H:%M:%S")
        self.log_lines.append(f"{stamp} {direction} {text}")
        noisy = text in ("PING", "PONG", "ACK MOVE", "ACK DEV WHEEL") or text.startswith(
            ("STATUS", "DEV RANGE", "DEV LINE", "DEV SYS", "DEV WHEEL", "MOVE"))
        if self.hide_noise.get() and noisy:
            return
        tag = "ERR" if text.startswith(("ERROR", "NAK")) else direction
        self.console.insert("end", f"{stamp} {direction} {text}\n", tag)
        if int(self.console.index("end-1c").split(".")[0]) > 1000:
            self.console.delete("1.0", "200.0")
        self.console.see("end")

    def _save_log(self):
        path = filedialog.asksaveasfilename(defaultextension=".log", initialfile=time.strftime("agv_%Y%m%d_%H%M%S.log"))
        if path:
            with open(path, "w", encoding="utf-8") as fp:
                fp.write("\n".join(self.log_lines))

    # ── 수신 처리 ─────────────────────────────────────────

    def on_dev(self, r: dict):
        kind = r["kind"]
        if kind == "RANGE":
            self.ranges.append(r["distance"])
            self.range_lbl.config(text=f"{r['distance']:.3f} m")
            self._draw_graph()
        elif kind == "LINE":
            self._draw_line(r["offset"])
            raw = r.get("raw_event", r["event"])
            self.line_lbl.config(text=f"오프셋 {r['offset']:+.2f}   이벤트 {r['event']}   (판정 {raw})")
        elif kind == "SYS":
            self.sys_lbl.config(text=f"온도 {r['temp']:.1f}℃ / CPU {r['cpu']:.0f}% / 메모리 {r['mem']:.0f}%")
        elif kind == "NODES":
            self.nodes.delete(0, "end")
            for n in r["nodes"]:
                self.nodes.insert("end", n)

    def update_state(self, state: str):
        self.state_lbl.config(text=f"차량 상태: {state}")

    def resend_watches(self):
        """재연결 시 차량은 WATCH를 모두 끄므로 켜 둔 항목을 다시 보낸다."""
        for k, var in self.watch.items():
            if var.get():
                self.send("DEV", "WATCH", k, "ON")

"""사용자 탭 — 초기 설정(트랙 매핑)·물품별 수량 입력·작업 시작·재개, 상태, 사건, 영상, 수동 제어 (변경 시나리오 2026-11-03)."""
import time
import tkinter as tk
from tkinter import ttk

from ui.manual_pad import ManualPad
from ui.video_view import VideoView

STATE_KO = {
    "IDLE": "대기", "NAVIGATE": "이동", "CHECK_SIGN": "표지판 확인", "APPROACH": "표지판 접근",
    "LOAD": "상차", "UNLOAD": "하차", "UTURN": "유턴", "PARKED": "도착 완료",
    "PAUSED_OBSTACLE": "장애물 정지", "ERROR": "오류", "ESTOP": "비상 정지", "MANUAL": "수동",
}
MODE_KO = {"NONE": "-", "SETUP": "초기 설정", "WORK": "작업"}
CARGO_KO = {"NONE": "없음", "CIRCLE": "적 (빨강 LED)", "SQUARE": "녹 (초록 LED)"}
TARGET_KO = {"NONE": "-", "WORK": "상차 장소", "DROP_A": "적 창고", "DROP_B": "녹 창고", "PARK": "Start"}
COLOR_KO = {"red": ("빨강", "#d32f2f"), "green": ("초록", "#2e7d32"), "blue": ("파랑", "#1565c0"),
            "yellow": ("노랑", "#f9a825")}
STATE_COLOR = {"ESTOP": "#d32f2f", "ERROR": "#d32f2f", "PAUSED_OBSTACLE": "#f57c00", "MANUAL": "#1976d2"}
MAP_ORDER = ("WORK", "DROP_A", "DROP_B", "PARK")


class UserTab(ttk.Frame):
    def __init__(self, master, send, on_map=None):
        super().__init__(master, padding=8)
        self.send = send
        self.on_map = on_map or (lambda m: None)        # 매핑 결과 저장 (MainWindow → config.json)
        self.track_map: dict = {}
        left = ttk.Frame(self)
        left.grid(row=0, column=0, sticky="nsw", padx=(0, 10))
        right = ttk.Frame(self)
        right.grid(row=0, column=1, sticky="n")

        setup = ttk.LabelFrame(left, text="초기 설정 (트랙 매핑)")
        setup.pack(fill="x", pady=(0, 6))
        ttk.Button(setup, text="초기 설정 시작", command=lambda: self.send("SETUP")).grid(
            row=0, column=0, columnspan=2, sticky="w", padx=4, pady=4)
        ttk.Label(setup, text="Start 표지판을 보고 세운 뒤 시작", foreground="gray").grid(
            row=0, column=2, sticky="w", padx=4)
        self.map_lbl = {}
        for r, target in enumerate(MAP_ORDER, start=1):
            ttk.Label(setup, text=TARGET_KO[target], width=14).grid(row=r, column=0, sticky="w", padx=4)
            self.map_lbl[target] = tk.Label(setup, text="?", width=8, font=("맑은 고딕", 11, "bold"), fg="gray")
            self.map_lbl[target].grid(row=r, column=1, sticky="w")

        job = ttk.LabelFrame(left, text="물류 운반 (물품별 0~99개, 1회 최대 9개씩)")
        job.pack(fill="x", pady=(0, 6))
        self.circle, self.square = tk.IntVar(value=3), tk.IntVar(value=2)
        for c, (text, var) in enumerate((("적", self.circle), ("녹", self.square))):
            ttk.Label(job, text=text).grid(row=0, column=2 * c, padx=(4, 2), pady=4)
            ttk.Spinbox(job, from_=0, to=99, textvariable=var, width=4).grid(row=0, column=2 * c + 1)
        self.start_btn = ttk.Button(job, text="작업 시작", command=self._start)
        self.start_btn.grid(row=0, column=4, padx=4)
        ttk.Button(job, text="재개", command=lambda: self.send("RESUME")).grid(row=0, column=5, padx=4)
        self.job_hint = ttk.Label(job, text="초기 설정을 먼저 하세요", foreground="#d32f2f")
        self.job_hint.grid(row=1, column=0, columnspan=6, sticky="w", padx=4)

        st = ttk.LabelFrame(left, text="상태")
        st.pack(fill="x", pady=(0, 6))
        self.state_lbl = tk.Label(st, text="연결 안 됨", font=("맑은 고딕", 18, "bold"), width=12, anchor="w")
        self.state_lbl.grid(row=0, column=0, columnspan=2, sticky="w", padx=4)
        self.progress = ttk.Progressbar(st, length=220, maximum=1)
        self.progress.grid(row=1, column=0, columnspan=2, sticky="w", padx=4, pady=2)
        self.info = {}
        rows = (("mode", "모드"), ("done", "진행"), ("cargo", "실은 화물"), ("load", "실은 개수"), ("target", "목적지"))
        for r, (key, label) in enumerate(rows, start=2):
            ttk.Label(st, text=label).grid(row=r, column=0, sticky="w", padx=4)
            self.info[key] = ttk.Label(st, text="-", font=("맑은 고딕", 11, "bold"))
            self.info[key].grid(row=r, column=1, sticky="w")

        self.pad = ManualPad(left, send)
        self.pad.pack(fill="x", pady=(0, 6))

        ev = ttk.LabelFrame(left, text="사건·오류")
        ev.pack(fill="both", expand=True)
        self.events = tk.Listbox(ev, height=7, width=42)
        self.events.pack(fill="both", expand=True, padx=2, pady=2)

        self.video = VideoView(right, 640, 480)
        self.video.pack()

    def _start(self):
        try:
            circle, square = int(self.circle.get()), int(self.square.get())
        except (ValueError, tk.TclError):
            self.add_event("수량은 0~99 숫자", error=True)
            return
        if not (0 <= circle <= 99 and 0 <= square <= 99) or circle + square == 0:
            self.add_event("수량은 물품별 0~99, 합계 1 이상", error=True)
            return
        self.send("START", circle, square)

    def update_map(self, track_map: dict, save: bool = True):
        self.track_map = {k: v for k, v in track_map.items() if v}
        if save:
            self.on_map(dict(self.track_map))
        for target, lbl in self.map_lbl.items():
            name, color = COLOR_KO.get(self.track_map.get(target), ("?", "gray"))
            lbl.config(text=name, fg=color)
        ready = all(self.track_map.get(t) for t in MAP_ORDER)
        self.job_hint.config(text="" if ready else "초기 설정을 먼저 하세요")

    def update_status(self, r: dict):
        state, mode = r["state"], r.get("mode", "NONE")
        self.state_lbl.config(text=STATE_KO.get(state, state), fg=STATE_COLOR.get(state, "#2e7d32"))
        self.progress.config(maximum=max(1, r["total"]), value=r["done"])
        unit = "지점" if mode == "SETUP" else "개"
        self.info["mode"].config(text=MODE_KO.get(mode, mode))
        self.info["done"].config(text=f"{r['done']} / {r['total']} {unit}")
        self.info["cargo"].config(text=CARGO_KO.get(r["cargo"], r["cargo"]))
        self.info["load"].config(text=str(r.get("load", 0)))
        target = r["target"]
        self.info["target"].config(text=COLOR_KO[target][0] + " 따라가는 중" if target in COLOR_KO
                                   else TARGET_KO.get(target, target))
        self.pad.set_manual_state(state == "MANUAL")
        self.video.set_overlay(f"{MODE_KO.get(mode, mode)} {state}  {r['done']}/{r['total']}  "
                               f"{r['cargo']}×{r.get('load', 0)} → {target}")

    def set_disconnected(self):
        self.state_lbl.config(text="연결 안 됨", fg="gray")
        self.video.set_overlay("연결 안 됨")

    def add_event(self, text: str, error: bool = False):
        self.events.insert(0, f"{time.strftime('%H:%M:%S')}  {text}")
        if error:
            self.events.itemconfig(0, fg="#d32f2f")
        if self.events.size() > 200:
            self.events.delete(200, "end")

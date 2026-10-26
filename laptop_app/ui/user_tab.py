"""사용자 탭 — 작업 입력·시작·재개, 상태, 사건, 영상, 수동 제어 (실제 운행용)."""
import time
import tkinter as tk
from tkinter import ttk

from ui.manual_pad import ManualPad
from ui.video_view import VideoView

STATE_KO = {
    "IDLE": "대기", "NAVIGATE": "이동", "AT_JUNCTION": "분기점", "CHECK_SIGN": "표지판 확인",
    "APPROACH": "접근", "LOAD": "적재", "UNLOAD": "하차", "UTURN": "유턴", "PARKED": "주차 완료",
    "PAUSED_OBSTACLE": "장애물 정지", "ERROR": "오류", "ESTOP": "비상 정지", "MANUAL": "수동",
}
CARGO_KO = {"NONE": "없음", "CIRCLE": "원 (빨강)", "SQUARE": "사각형 (초록)"}
TARGET_KO = {"NONE": "-", "WORK": "작업장", "DROP_A": "하차 A", "DROP_B": "하차 B", "PARK": "대기장소"}
STATE_COLOR = {"ESTOP": "#d32f2f", "ERROR": "#d32f2f", "PAUSED_OBSTACLE": "#f57c00", "MANUAL": "#1976d2"}


class UserTab(ttk.Frame):
    def __init__(self, master, send):
        super().__init__(master, padding=8)
        self.send = send
        left = ttk.Frame(self)
        left.grid(row=0, column=0, sticky="nsw", padx=(0, 10))
        right = ttk.Frame(self)
        right.grid(row=0, column=1, sticky="n")

        job = ttk.LabelFrame(left, text="작업")
        job.pack(fill="x", pady=(0, 6))
        ttk.Label(job, text="작업 횟수").grid(row=0, column=0, padx=4, pady=4)
        self.count = tk.IntVar(value=3)
        ttk.Spinbox(job, from_=1, to=9, textvariable=self.count, width=4).grid(row=0, column=1)
        ttk.Button(job, text="작업 시작", command=self._start).grid(row=0, column=2, padx=4)
        ttk.Button(job, text="재개", command=lambda: self.send("RESUME")).grid(row=0, column=3, padx=4)

        st = ttk.LabelFrame(left, text="상태")
        st.pack(fill="x", pady=(0, 6))
        self.state_lbl = tk.Label(st, text="연결 안 됨", font=("맑은 고딕", 18, "bold"), width=12, anchor="w")
        self.state_lbl.grid(row=0, column=0, columnspan=2, sticky="w", padx=4)
        self.progress = ttk.Progressbar(st, length=220, maximum=1)
        self.progress.grid(row=1, column=0, columnspan=2, sticky="w", padx=4, pady=2)
        self.info = {}
        for r, (key, label) in enumerate((("done", "진행"), ("cargo", "적재 화물"), ("target", "목적지")), start=2):
            ttk.Label(st, text=label).grid(row=r, column=0, sticky="w", padx=4)
            self.info[key] = ttk.Label(st, text="-", font=("맑은 고딕", 11, "bold"))
            self.info[key].grid(row=r, column=1, sticky="w")

        self.pad = ManualPad(left, send)
        self.pad.pack(fill="x", pady=(0, 6))

        ev = ttk.LabelFrame(left, text="사건·오류")
        ev.pack(fill="both", expand=True)
        self.events = tk.Listbox(ev, height=9, width=42)
        self.events.pack(fill="both", expand=True, padx=2, pady=2)

        self.video = VideoView(right, 640, 480)
        self.video.pack()

    def _start(self):
        try:
            self.send("START", int(self.count.get()))
        except (ValueError, tk.TclError):
            self.add_event("작업 횟수는 1~9", error=True)

    def update_status(self, r: dict):
        state = r["state"]
        self.state_lbl.config(text=STATE_KO.get(state, state), fg=STATE_COLOR.get(state, "#2e7d32"))
        self.progress.config(maximum=max(1, r["total"]), value=r["done"])
        self.info["done"].config(text=f"{r['done']} / {r['total']}")
        self.info["cargo"].config(text=CARGO_KO.get(r["cargo"], r["cargo"]))
        self.info["target"].config(text=TARGET_KO.get(r["target"], r["target"]))
        self.pad.set_manual_state(state == "MANUAL")
        self.video.set_overlay(f"{state}  {r['done']}/{r['total']}  {r['cargo']} → {r['target']}")

    def set_disconnected(self):
        self.state_lbl.config(text="연결 안 됨", fg="gray")
        self.video.set_overlay("연결 안 됨")

    def add_event(self, text: str, error: bool = False):
        self.events.insert(0, f"{time.strftime('%H:%M:%S')}  {text}")
        if error:
            self.events.itemconfig(0, fg="#d32f2f")
        if self.events.size() > 200:
            self.events.delete(200, "end")

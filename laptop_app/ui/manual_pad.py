"""수동 제어 — 수동 모드 켜기/끄기, 방향 버튼·W/A/S/D → MOVE v w 를 0.3초마다 반복."""
import tkinter as tk
from tkinter import ttk

import _paths  # noqa: F401
from agv_protocol.messages import MOVE_REPEAT_S

KEYS = {"w": (1, 0), "s": (-1, 0), "a": (0, 1), "d": (0, -1)}


class ManualPad(ttk.LabelFrame):
    def __init__(self, master, send):
        super().__init__(master, text="수동 제어")
        self.send = send                    # send(kind, *args) -> bool
        self.manual = tk.BooleanVar(value=False)
        self.speed = tk.DoubleVar(value=0.5)
        self.pressed: set[str] = set()
        self._repeat_job = None

        ttk.Checkbutton(self, text="수동 모드", variable=self.manual, command=self._toggle).grid(
            row=0, column=0, columnspan=3, sticky="w", padx=4, pady=2)
        pad = ttk.Frame(self)
        pad.grid(row=1, column=0, columnspan=3, pady=2)
        for key, label, r, c in (("w", "▲ W", 0, 1), ("a", "◀ A", 1, 0), ("s", "▼ S", 1, 1), ("d", "D ▶", 1, 2)):
            b = ttk.Button(pad, text=label, width=6)
            b.grid(row=r, column=c, padx=2, pady=2)
            b.bind("<ButtonPress-1>", lambda e, k=key: self.press(k))
            b.bind("<ButtonRelease-1>", lambda e, k=key: self.release(k))
        ttk.Label(self, text="속도").grid(row=2, column=0, sticky="e")
        ttk.Scale(self, from_=0.1, to=1.0, variable=self.speed, length=140).grid(row=2, column=1, columnspan=2)
        ttk.Label(self, text="키보드 W/A/S/D (사용자 탭에서)", foreground="gray").grid(
            row=3, column=0, columnspan=3, sticky="w", padx=4)

    def _toggle(self):
        if not self.send("MANUAL", "ON" if self.manual.get() else "OFF"):
            self.manual.set(False)

    def set_manual_state(self, on: bool):
        self.manual.set(on)
        if not on:
            self.pressed.clear()

    def press(self, key):
        if not self.manual.get() or key in self.pressed:
            return
        self.pressed.add(key)
        self._send_move()

    def release(self, key):
        if key in self.pressed:
            self.pressed.discard(key)
            self._send_move()

    def _vector(self):
        v = sum(KEYS[k][0] for k in self.pressed) * self.speed.get()
        w = sum(KEYS[k][1] for k in self.pressed) * self.speed.get()
        return round(max(-1, min(1, v)), 2), round(max(-1, min(1, w)), 2)

    def _send_move(self):
        if self._repeat_job:
            self.after_cancel(self._repeat_job)
            self._repeat_job = None
        v, w = self._vector()
        self.send("MOVE", float(v), float(w))
        if self.pressed:                     # 누르고 있는 동안 반복 (끊기면 차량 워치독이 정지)
            self._repeat_job = self.after(int(MOVE_REPEAT_S * 1000), self._send_move)

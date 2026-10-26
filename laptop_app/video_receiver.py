"""채널 2(영상) 수신 — 길이+JPEG 프레임을 모아 최신 1장만 보관, FPS·전송 속도 측정."""
import socket
import threading
import time

import _paths  # noqa: F401
from agv_protocol.framing import FrameReader
from bt_client import open_socket


class VideoReceiver:
    def __init__(self):
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()
        self._latest: bytes | None = None
        self._new = False
        self._running = False
        self._frames = 0
        self._bytes = 0
        self._window_start = time.monotonic()
        self.fps = 0.0
        self.kbps = 0.0
        self.error: str | None = None

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def connect(self, target: dict) -> None:
        self._sock = open_socket(target, video=True)
        self._running = True
        self.error = None
        threading.Thread(target=self._rx_loop, args=(self._sock,), daemon=True).start()

    def close(self) -> None:
        self._running = False
        sock, self._sock = self._sock, None
        if sock:
            try:
                sock.close()
            except OSError:
                pass
        self.fps = self.kbps = 0.0

    def take_new(self) -> bytes | None:
        """새 프레임이 있으면 반환(한 번만), 없으면 None."""
        with self._lock:
            if not self._new:
                return None
            self._new = False
            return self._latest

    @property
    def latest(self) -> bytes | None:
        with self._lock:
            return self._latest

    def _rx_loop(self, sock: socket.socket) -> None:
        reader = FrameReader()
        while self._running:
            try:
                data = sock.recv(65536)
            except OSError as e:
                self.error = str(e)
                break
            if not data:
                break
            self._bytes += len(data)
            try:
                frames = reader.feed(data)
            except ValueError as e:
                self.error = str(e)
                break
            if frames:
                self._frames += len(frames)
                with self._lock:
                    self._latest, self._new = frames[-1], True
            now = time.monotonic()
            if now - self._window_start >= 1.0:
                span = now - self._window_start
                self.fps, self.kbps = self._frames / span, self._bytes * 8 / 1000 / span
                self._frames = self._bytes = 0
                self._window_start = now
        if self._sock is sock:
            self.close()

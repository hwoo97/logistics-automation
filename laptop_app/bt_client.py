"""채널 1(제어) 클라이언트 — 연결, 수신 스레드, 0.5초 PING, 명령 전송.

콜백은 수신 스레드에서 불리므로 화면 갱신은 호출한 쪽(MainWindow)이 큐로 넘겨 처리한다.
target 예: {"transport": "rfcomm", "addr": "2C:CF:67:2C:DB:A9"} / {"transport": "tcp", "host": "127.0.0.1", "port": 9000}
"""
import socket
import threading
import time
from typing import Callable

import _paths  # noqa: F401
from agv_protocol.messages import CONTROL_CHANNEL, PING_INTERVAL_S, VIDEO_CHANNEL, format_command


def open_socket(target: dict, video: bool = False, timeout: float = 10.0) -> socket.socket:
    if target["transport"] == "tcp":
        port = target["port"] + (1 if video else 0)
        sock = socket.create_connection((target["host"], port), timeout=timeout)
    else:
        sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        sock.settimeout(timeout)
        sock.connect((target["addr"], VIDEO_CHANNEL if video else CONTROL_CHANNEL))
    sock.settimeout(None)
    return sock


class BtClient:
    def __init__(self, on_line: Callable[[str, str], None], on_closed: Callable[[str], None]):
        self.on_line = on_line              # (방향 "TX"/"RX", 텍스트)
        self.on_closed = on_closed          # (이유)
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()
        self._running = False
        self._ping_sent = 0.0
        self.rtt_ms: float | None = None
        self.last_rx = 0.0

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def connect(self, target: dict) -> None:
        """연결 실패 시 OSError 그대로 올림 (호출한 쪽에서 안내)."""
        self._sock = open_socket(target)
        self._running = True
        self.last_rx = time.monotonic()
        threading.Thread(target=self._rx_loop, args=(self._sock,), daemon=True).start()
        threading.Thread(target=self._ping_loop, daemon=True).start()

    def close(self, reason: str = "사용자 해제") -> None:
        self._running = False
        sock, self._sock = self._sock, None
        if sock:
            try:
                sock.close()
            except OSError:
                pass
            self.on_closed(reason)

    def send_line(self, line: str) -> bool:
        sock = self._sock
        if sock is None:
            return False
        try:
            with self._lock:
                sock.sendall(line.encode("utf-8") + b"\n")
        except OSError as e:
            self.close(f"전송 실패: {e}")
            return False
        if line == "PING":
            self._ping_sent = time.monotonic()
        else:
            self.on_line("TX", line)
        return True

    def send(self, kind: str, *args) -> bool:
        """format_command로 검증 후 전송 (형식 오류는 ValueError)."""
        return self.send_line(format_command(kind, *args))

    def _ping_loop(self) -> None:
        while self._running:
            self.send_line("PING")
            time.sleep(PING_INTERVAL_S)

    def _rx_loop(self, sock: socket.socket) -> None:
        buf = b""
        reason = "차량이 연결을 끊음"
        while self._running:
            try:
                data = sock.recv(4096)
            except OSError as e:
                reason = f"수신 오류: {e}"
                break
            if not data:
                break
            self.last_rx = time.monotonic()
            buf += data
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                text = raw.decode("utf-8", errors="replace").strip()
                if not text:
                    continue
                if text == "PONG" and self._ping_sent:
                    self.rtt_ms = (time.monotonic() - self._ping_sent) * 1000
                self.on_line("RX", text)
        if self._sock is sock:
            self.close(reason)

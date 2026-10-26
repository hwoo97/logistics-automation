"""RFCOMM 서버 — 채널 1(제어 텍스트)·채널 2(영상) — class_design.md 4.2절.

ROS에 의존하지 않는 순수 Python. bt_bridge_node와 노트북 앱의 가짜 차량(fake_vehicle)이 함께 쓴다.
transport="tcp"이면 블루투스 대신 TCP(제어 = port, 영상 = port + 1)로 대기한다 (PC 시험용).
"""
import socket
import threading
import time
from typing import Callable

from agv_protocol.framing import pack_frame
from agv_protocol.messages import CONTROL_CHANNEL, HEARTBEAT_TIMEOUT_S, VIDEO_CHANNEL


class RfcommServer:
    def __init__(self, on_line: Callable[[str], None],
                 on_connect: Callable[[], None] | None = None,
                 on_disconnect: Callable[[], None] | None = None,
                 transport: str = "rfcomm",
                 control_channel: int = CONTROL_CHANNEL, video_channel: int = VIDEO_CHANNEL,
                 tcp_host: str = "0.0.0.0", tcp_port: int = 9000):
        self.on_line = on_line
        self.on_connect = on_connect
        self.on_disconnect = on_disconnect
        self._transport = transport
        self._addrs = ((control_channel, tcp_port), (video_channel, tcp_port + 1))
        self._tcp_host = tcp_host
        self._ctrl: socket.socket | None = None
        self._video: socket.socket | None = None
        self._send_lock = threading.Lock()
        self._frame_cv = threading.Condition()
        self._pending_frame: bytes | None = None
        self._running = False
        self._threads: list[threading.Thread] = []
        self.last_rx = 0.0
        self.ever_connected = False

    # ── 시작·종료 ─────────────────────────────────────────

    def _listen(self, index: int) -> socket.socket:
        channel, port = self._addrs[index]
        if self._transport == "tcp":
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((self._tcp_host, port))
        else:
            srv = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            srv.bind((socket.BDADDR_ANY, channel))
        srv.listen(1)
        srv.settimeout(0.5)                     # stop()이 수락 대기를 끝낼 수 있게
        return srv

    def start(self) -> None:
        self._running = True
        listeners = [self._listen(0), self._listen(1)]
        for target, srv in ((self._control_loop, listeners[0]), (self._video_accept_loop, listeners[1])):
            t = threading.Thread(target=target, args=(srv,), daemon=True)
            t.start()
            self._threads.append(t)
        t = threading.Thread(target=self._video_send_loop, daemon=True)
        t.start()
        self._threads.append(t)

    def stop(self) -> None:
        self._running = False
        with self._frame_cv:
            self._frame_cv.notify_all()
        for s in (self._ctrl, self._video):
            if s:
                try:
                    s.close()
                except OSError:
                    pass
        for t in self._threads:
            t.join(timeout=1.0)

    @property
    def where(self) -> str:
        (ch, port), (vch, vport) = self._addrs
        if self._transport == "tcp":
            return f"TCP {self._tcp_host}:{port} (영상 {vport})"
        return f"RFCOMM 채널 {ch} (영상 {vch})"

    # ── 상태 ─────────────────────────────────────────────

    @property
    def connected(self) -> bool:
        return self._ctrl is not None

    def is_alive(self, now: float | None = None) -> bool:
        """마지막 수신 후 HEARTBEAT_TIMEOUT_S 이내인지."""
        now = time.monotonic() if now is None else now
        return self.connected and now - self.last_rx <= HEARTBEAT_TIMEOUT_S

    # ── 제어 채널 ─────────────────────────────────────────

    def _accept(self, srv: socket.socket) -> socket.socket | None:
        while self._running:
            try:
                conn, _ = srv.accept()
                conn.settimeout(None)
                return conn
            except socket.timeout:
                continue
            except OSError:
                return None
        return None

    def _control_loop(self, srv: socket.socket) -> None:
        with srv:
            while self._running:
                conn = self._accept(srv)
                if conn is None:
                    break
                old, self._ctrl = self._ctrl, conn       # 새 연결이 오면 이전 연결을 대체
                if old:
                    old.close()                          # 멈춘 이전 클라이언트가 채널을 붙잡지 않게
                self.last_rx = time.monotonic()
                self.ever_connected = True
                if self.on_connect:
                    self.on_connect()
                threading.Thread(target=self._serve_control, args=(conn,), daemon=True).start()

    def _serve_control(self, conn: socket.socket) -> None:
        self._read_lines(conn)
        if self._ctrl is conn:
            self._ctrl = None
            if self.on_disconnect:
                self.on_disconnect()

    def _read_lines(self, conn: socket.socket) -> None:
        buf = b""
        while self._running:
            try:
                data = conn.recv(1024)
            except OSError:
                return
            if not data:
                return
            self.last_rx = time.monotonic()
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", errors="replace").strip()
                if text:
                    self.on_line(text)

    def send_line(self, text: str) -> bool:
        """한 줄 전송. 연결이 없으면 무시하고 False."""
        conn = self._ctrl
        if conn is None:
            return False
        try:
            with self._send_lock:
                conn.sendall(text.encode("utf-8") + b"\n")
            return True
        except OSError:
            return False

    # ── 영상 채널 ─────────────────────────────────────────

    def _video_accept_loop(self, srv: socket.socket) -> None:
        with srv:
            while self._running:
                conn = self._accept(srv)
                if conn is None:
                    break
                old, self._video = self._video, conn
                if old:
                    old.close()

    def send_frame(self, jpeg: bytes) -> bool:
        """최신 프레임 1장만 보관 — 이전 전송이 안 끝났으면 덮어써서 밀린 프레임을 버린다."""
        if self._video is None:
            return False
        with self._frame_cv:
            self._pending_frame = jpeg
            self._frame_cv.notify()
        return True

    def _video_send_loop(self) -> None:
        while self._running:
            with self._frame_cv:
                while self._running and self._pending_frame is None:
                    self._frame_cv.wait(0.5)
                jpeg, self._pending_frame = self._pending_frame, None
            conn = self._video
            if jpeg is None or conn is None:
                continue
            try:
                conn.sendall(pack_frame(jpeg))
            except OSError:
                if self._video is conn:
                    self._video = None

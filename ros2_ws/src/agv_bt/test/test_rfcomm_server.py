"""RfcommServer를 TCP 모드로 띄워 제어·영상 채널 동작을 확인한다 (블루투스 없이)."""
import socket
import time

import pytest

from agv_bt.rfcomm_server import RfcommServer
from agv_protocol.framing import FrameReader

PORT = 19500


def wait_for(cond, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


@pytest.fixture
def server():
    lines, events = [], []
    srv = RfcommServer(on_line=lines.append, on_connect=lambda: events.append("connect"),
                       on_disconnect=lambda: events.append("disconnect"),
                       transport="tcp", tcp_host="127.0.0.1", tcp_port=PORT)
    srv.start()
    yield srv, lines, events
    srv.stop()


def connect(port):
    return socket.create_connection(("127.0.0.1", port), timeout=2)


def test_control_lines_and_reply(server):
    srv, lines, events = server
    assert not srv.connected and srv.send_line("X") is False
    c = connect(PORT)
    assert wait_for(lambda: srv.connected)
    c.sendall(b"PING\nSTART 3\n\n")
    assert wait_for(lambda: lines == ["PING", "START 3"])
    assert srv.send_line("PONG")
    assert c.recv(100) == b"PONG\n"
    assert srv.is_alive()
    c.close()
    assert wait_for(lambda: events == ["connect", "disconnect"])
    assert not srv.connected and srv.ever_connected


def test_heartbeat_timeout(server):
    srv, _, _ = server
    c = connect(PORT)
    assert wait_for(lambda: srv.connected)
    assert srv.is_alive()
    assert not srv.is_alive(now=srv.last_rx + 1.6)
    c.close()


def test_new_client_replaces_old(server):
    srv, lines, _ = server
    a = connect(PORT)
    assert wait_for(lambda: srv.connected)
    b = connect(PORT)
    b.sendall(b"PING\n")
    assert wait_for(lambda: lines == ["PING"])
    assert srv.send_line("PONG") and b.recv(100) == b"PONG\n"
    a.close()
    b.close()


def test_video_latest_frame(server):
    srv, _, _ = server
    assert srv.send_frame(b"none") is False
    v = connect(PORT + 1)
    assert wait_for(lambda: srv._video is not None)
    assert srv.send_frame(b"\xff\xd8frame1\xff\xd9")
    reader, got = FrameReader(), []
    v.settimeout(2)
    while not got:
        got += reader.feed(v.recv(4096))
    assert got[0] == b"\xff\xd8frame1\xff\xd9"
    v.close()

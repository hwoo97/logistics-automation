import numpy as np
import pytest

from agv_bt.bridge_core import BridgeCore
from agv_bt.sysinfo import SysMonitor, parse_cpu_times, parse_mem_percent, parse_temp
from agv_protocol.messages import parse_reply


class FakeIO:
    def __init__(self):
        self.sent, self.calls = [], []

    def send(self, line):
        self.sent.append(line)

    def __getattr__(self, name):                       # estop, cmd_vel, led, pose, servo, wheels, mission
        return lambda *a: self.calls.append((name, *a))

    def node_names(self):
        return ["led_node", "camera"]

    def replies(self, kind=None):
        r = [parse_reply(s) for s in self.sent]
        return [x for x in r if kind is None or x["type"] == kind]


@pytest.fixture
def core():
    io = FakeIO()
    return BridgeCore(io, standalone=True), io


def test_ping(core):
    c, io = core
    c.on_line("PING")
    assert io.sent == ["PONG"]


def test_dev_commands_reach_io(core):
    c, io = core
    for line in ("DEV LED RED", "DEV POSE SIGN", "DEV SERVO 120 60", "DEV WHEEL 0.3 -0.3 0 0", "DEV SPEED 0.7",
                 "DEV NODES"):
        c.on_line(line)
    assert ("led", "RED") in io.calls and ("pose", "SIGN") in io.calls
    assert ("servo", 120.0, 60.0) in io.calls and ("wheels", [0.3, -0.3, 0.0, 0.0]) in io.calls
    assert ("speed", 0.7) in io.calls
    assert io.sent[-1] == "DEV NODES camera,led_node"
    assert [r["command"] for r in io.replies("ACK")] == ["DEV LED", "DEV POSE", "DEV SERVO", "DEV WHEEL", "DEV SPEED",
                                                         "DEV NODES"]


def test_target_reaches_io(core):
    c, io = core
    c.on_line("DEV TARGET RED")
    assert ("target", "RED") in io.calls


def test_watch(core):
    c, _ = core
    c.on_line("DEV WATCH ALL ON")
    assert c.watches == {"RANGE", "LINE", "MARKER", "SYS"}
    c.on_line("DEV WATCH LINE OFF")
    assert c.watches == {"RANGE", "MARKER", "SYS"}


def test_invalid_and_start_rejected(core):
    c, io = core
    c.on_line("DEV LED BLUE")
    c.on_line("START 3 2")
    c.on_line("SETUP")
    naks = io.replies("NAK")
    assert naks[0]["command"] == "DEV LED" and "OFF/RED" in naks[0]["reason"]
    assert naks[1]["command"] == "START" and "mission_node" in naks[1]["reason"]
    assert naks[2]["command"] == "SETUP" and "mission_node" in naks[2]["reason"]


def test_stop_resume(core):
    c, io = core
    c.on_line("STOP")
    assert ("estop", True) in io.calls and c.state == "ESTOP"
    c.on_line("DEV WHEEL 0.5 0 0 0")                   # ESTOP 중 바퀴 거부
    assert io.replies("NAK")[-1]["command"] == "DEV WHEEL"
    c.on_line("DEV LED GREEN")                         # LED는 허용
    assert ("led", "GREEN") in io.calls
    c.on_line("RESUME")
    assert ("estop", False) in io.calls and c.state == "IDLE"
    assert io.replies("EVENT")[-1]["event"] == "RESUMED"


def test_drive_follow_and_stop(core):
    c, io = core
    c.on_line("DEV DRIVE FOLLOW")
    assert ("drive", "FOLLOW") in io.calls and io.replies("ACK")[-1]["command"] == "DEV DRIVE"
    c.set_driving(True)
    for line in ("MANUAL ON", "DEV WHEEL 0.5 0 0 0", "DEV DRIVE FOLLOW"):     # 주행 중 겹치는 명령 거부
        c.on_line(line)
        assert "라인 주행 중" in io.replies("NAK")[-1]["reason"]
    c.on_line("DEV SPEED 0.7")                                              # 속도·조정은 허용
    c.on_line("DEV TUNE TURN 0.5")
    assert ("speed", 0.7) in io.calls and ("tune", "TURN", 0.5) in io.calls
    c.on_line("DEV DRIVE STOP")
    assert ("drive", "STOP") in io.calls


def test_drive_uturn(core):
    c, io = core
    c.on_line("DEV DRIVE UTURN")
    assert ("drive", "UTURN") in io.calls
    c.set_driving(True)
    c.on_line("DEV DRIVE UTURN")                                            # 동작 중 겹치는 명령 거부
    assert "라인 주행 중" in io.replies("NAK")[-1]["reason"]


def test_drive_follow_rejected_outside_idle(core):
    c, io = core
    c.on_line("MANUAL ON")
    c.on_line("DEV DRIVE FOLLOW")
    assert io.replies("NAK")[-1]["command"] == "DEV DRIVE" and ("drive", "FOLLOW") not in io.calls


def test_manual_move(core):
    c, io = core
    c.on_line("MOVE 0.5 0")
    assert io.replies("NAK")[-1]["command"] == "MOVE"   # IDLE에서는 거부
    c.on_line("MANUAL ON")
    c.on_line("MOVE 0.5 -0.2")
    assert ("cmd_vel", 0.5, -0.2) in io.calls and c.state == "MANUAL"
    c.on_line("MANUAL OFF")
    assert io.calls[-1] == ("cmd_vel", 0.0, 0.0) and c.state == "IDLE"


def test_heartbeat(core):
    c, io = core
    c.on_heartbeat(ever_connected=False, alive=False)  # 첫 연결 전: 감시 안 함
    assert c.state == "IDLE" and not io.calls
    c.on_line("DEV WATCH RANGE ON")
    c.on_heartbeat(ever_connected=True, alive=False)
    assert c.state == "ESTOP" and ("estop", True) in io.calls and not c.watches
    c.on_heartbeat(ever_connected=True, alive=False)   # 중복 처리 없음
    assert io.calls.count(("estop", True)) == 1
    io.sent.clear()
    c.on_connect()
    assert [r["type"] for r in io.replies()] == ["STATUS", "MAP", "ERROR"]
    c.on_line("RESUME")
    c.on_connect()
    assert io.replies("ERROR") and len(io.replies("ERROR")) == 1   # 재개 후에는 E_LINK 없음


def test_mission_mode_forwards():
    io = FakeIO()
    c = BridgeCore(io, standalone=False)
    c.on_line("START 3 2")                             # 매핑 전에는 거부
    assert io.replies("NAK")[-1]["reason"] == "초기 설정 필요" and not io.calls
    c.on_line("SETUP")
    assert io.calls[-1][1].line() == "SETUP"
    track = {"WORK": "blue", "DROP_A": "red", "DROP_B": "green", "PARK": "yellow"}
    c.on_mission_status("NAVIGATE", "WORK", 0, 5, "NONE", 0, "WORK", track)
    assert c.state == "NAVIGATE" and io.sent[-2] == "MAP WORK=blue DROP_A=red DROP_B=green PARK=yellow"
    assert io.sent[-1] == "STATUS NAVIGATE WORK 0/5 NONE 0 WORK"
    n = len(io.sent)
    c.on_mission_status("NAVIGATE", "WORK", 0, 5, "NONE", 0, "WORK", track)      # 같으면 보내지 않음
    assert len(io.sent) == n
    c.on_mission_status("NAVIGATE", "WORK", 0, 5, "CIRCLE", 1, "WORK", track)    # 개수만 바뀌어도 보냄
    assert io.sent[-1] == "STATUS NAVIGATE WORK 0/5 CIRCLE 1 WORK"
    c.on_mission_status("IDLE", "NONE", 0, 0, "NONE", 0, "NONE", track)
    c.on_line("START 3 2")                             # 매핑 후에는 넘김
    assert io.calls[-1][0] == "mission" and io.calls[-1][1].line() == "START 3 2"
    c.on_mission_status("UTURN", "WORK", 0, 5, "NONE", 0, "WORK", track)
    c.on_line("DEV LED RED")                           # 작업 중 DEV 거부
    assert io.replies("NAK")[-1]["command"] == "DEV LED"
    c.on_line("STOP")
    assert ("estop", True) in io.calls and io.calls[-1][1].kind == "STOP"


def test_sysinfo_parsers():
    assert parse_temp("52123\n") == pytest.approx(52.123)
    assert parse_cpu_times("cpu  100 0 50 800 50 0 0 0 0 0\ncpu0 1 2 3") == (1000, 850)
    assert parse_mem_percent("MemTotal: 1000 kB\nMemFree: 100 kB\nMemAvailable: 600 kB\n") == pytest.approx(40.0)
    files = {"/sys/class/thermal/thermal_zone0/temp": "50000",
             "/proc/meminfo": "MemTotal: 1000 kB\nMemAvailable: 500 kB"}
    stats = iter(["cpu 100 0 0 900 0", "cpu 150 0 0 950 0"])
    mon = SysMonitor(read=lambda p: next(stats) if p == "/proc/stat" else files[p])
    assert mon.sample() == (50.0, 0.0, 50.0)
    assert mon.sample()[1] == pytest.approx(50.0)


def test_video_streamer():
    cv2 = pytest.importorskip("cv2")
    from agv_bt.video_streamer import VideoStreamer
    s = VideoStreamer(fps=5, size=(320, 240), quality=50)
    assert s.next_frame(0.0) is None
    img = np.zeros((480, 640, 3), np.uint8)
    img[:, 300:340] = 255
    s.update("bgr8", 480, 640, 640 * 3, img.tobytes())
    jpeg = s.next_frame(1.0)
    assert jpeg[:2] == b"\xff\xd8"
    assert cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR).shape == (240, 320, 3)
    s.update("bgr8", 480, 640, 640 * 3, img.tobytes())
    assert s.next_frame(1.1) is None                    # 0.2초 간격 전
    assert s.next_frame(1.25) is not None


def test_arrive_test_goes_to_mission():
    """도착 시험(개발자 탭) — mission_node TEST_ARRIVE, 정지는 주행 정지 + mission CANCEL (2026-11-06)."""
    io = FakeIO()
    c = BridgeCore(io, standalone=False)
    c.on_mission_status("IDLE", "NONE", 0, 0, "NONE", 0, "NONE", {})
    c.on_line("DEV DRIVE ARRIVE")
    sent = [a[1] for a in io.calls if a[0] == "mission"]
    assert io.replies("ACK")[-1]["command"] == "DEV DRIVE" and [m.kind for m in sent] == ["TEST_ARRIVE"]
    assert not any(a[0] == "drive" for a in io.calls)
    c.on_line("DEV DRIVE STOP")
    assert ("drive", "STOP") in io.calls and [a[1].kind for a in io.calls if a[0] == "mission"][-1] == "CANCEL"


def test_arrive_test_needs_mission_node(core):
    c, io = core
    c.on_line("DEV DRIVE ARRIVE")
    assert io.replies("NAK")[-1]["command"] == "DEV DRIVE"
    assert not any(a[0] in ("mission", "drive") for a in io.calls)


def test_standalone_drive_stop_does_not_touch_mission(core):
    c, io = core
    c.on_line("DEV DRIVE STOP")
    assert ("drive", "STOP") in io.calls and not any(a[0] == "mission" for a in io.calls)


def test_calibration_command_and_reply(core):
    """카메라 팬 정면 자동 맞춤 — DEV CAL AUTO → io.calibrate, pantilt_node 결과는 DEV CAL 줄로 (2026-11-06)."""
    c, io = core
    c.on_line("DEV CAL auto")
    assert ("calibrate", "AUTO") in io.calls and io.replies("ACK")[-1]["command"] == "DEV CAL"
    c.on_calibration("FAIL 라인이 안 보임")
    r = io.replies("DEV")[-1]
    assert r["kind"] == "CAL" and r["state"] == "FAIL" and r["detail"] == "라인이 안 보임"


def test_calibration_only_when_idle(core):
    c, io = core
    c.on_line("MANUAL ON")
    c.on_line("DEV CAL AUTO")
    assert io.replies("NAK")[-1]["command"] == "DEV CAL" and ("calibrate", "AUTO") not in io.calls


def test_dev_cal_sign_tilt_goes_to_pantilt(core):
    """DEV CAL SIGN 165 — 표지판 볼 때 틸트를 Pi에 저장 (2026-11-06)."""
    c, io = core
    c.on_line("DEV CAL sign 165")
    assert ("calibrate", "SIGN 165") in io.calls
    c.on_calibration("SIGN 165")
    r = io.replies("DEV")[-1]
    assert r["kind"] == "CAL" and r["state"] == "SIGN" and r["detail"] == "165"


def test_dev_reset_goes_to_mission_in_any_state():
    """시험 초기화 (2026-11-09) — 주행 정지 + mission_node RESET, 비상 정지 중에도."""
    io = FakeIO()
    c = BridgeCore(io, standalone=False)
    c.on_mission_status("ESTOP", "SETUP", 0, 4, "NONE", 0, "red", {})
    c.on_line("DEV RESET")
    assert io.replies("ACK")[-1]["command"] == "DEV RESET"
    assert ("drive", "STOP") in io.calls and [a[1].kind for a in io.calls if a[0] == "mission"] == ["RESET"]


def test_dev_reset_standalone_goes_idle(core):
    c, io = core
    c.on_line("STOP")
    c.on_line("DEV RESET")
    assert c.state == "IDLE" and ("estop", False) in io.calls
    assert io.replies("EVENT")[-1]["event"] == "RESET"

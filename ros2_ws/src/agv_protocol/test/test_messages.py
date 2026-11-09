import pytest

from agv_protocol.messages import (
    Command, check_allowed, command_key, format_ack, format_command, format_dev, format_error,
    format_event, format_map, format_nak, format_status, parse_command, parse_reply,
)
from agv_protocol.types import Cargo, Mode, State, Target


@pytest.mark.parametrize("line, kind, args", [
    ("START 3 2", "START", ["3", "2"]),
    ("START 0 9", "START", ["0", "9"]),
    ("START 12 5", "START", ["12", "5"]),
    ("START 99 0", "START", ["99", "0"]),
    ("setup", "SETUP", []),
    ("stop", "STOP", []),
    ("MANUAL on", "MANUAL", ["ON"]),
    ("MOVE 0.5 -0.2", "MOVE", ["0.5", "-0.2"]),
    ("DEV led RED", "DEV", ["LED", "RED"]),
    ("DEV SERVO 90 60", "DEV", ["SERVO", "90", "60"]),
    ("DEV WHEEL 0.3 0.3 0 -1", "DEV", ["WHEEL", "0.3", "0.3", "0", "-1"]),
    ("DEV WATCH ALL ON", "DEV", ["WATCH", "ALL", "ON"]),
    ("DEV NODES", "DEV", ["NODES"]),
    ("DEV speed 0.55", "DEV", ["SPEED", "0.55"]),
    ("DEV drive follow", "DEV", ["DRIVE", "FOLLOW"]),
    ("DEV DRIVE STOP", "DEV", ["DRIVE", "STOP"]),
    ("DEV tune turn 0.6", "DEV", ["TUNE", "TURN", "0.6"]),
    ("DEV TUNE CENTER -0.1", "DEV", ["TUNE", "CENTER", "-0.1"]),
    ("DEV tune confirm 3", "DEV", ["TUNE", "CONFIRM", "3"]),
    ("DEV TUNE LINE_END 10", "DEV", ["TUNE", "LINE_END", "10"]),
    ("DEV TUNE FORWARD 0.4", "DEV", ["TUNE", "FORWARD", "0.4"]),
    ("DEV TUNE BLIND 2.0", "DEV", ["TUNE", "BLIND", "2.0"]),
    ("DEV TUNE APPROACH 0.3", "DEV", ["TUNE", "APPROACH", "0.3"]),
    ("DEV TUNE SPIN 0.6", "DEV", ["TUNE", "SPIN", "0.6"]),
    ("DEV drive uturn", "DEV", ["DRIVE", "UTURN"]),
    ("DEV target red", "DEV", ["TARGET", "RED"]),
    ("DEV TARGET NONE", "DEV", ["TARGET", "NONE"]),
    ("DEV WATCH MARKER ON", "DEV", ["WATCH", "MARKER", "ON"]),
])
def test_parse_valid(line, kind, args):
    cmd = parse_command(line)
    assert (cmd.kind, cmd.args) == (kind, args)


@pytest.mark.parametrize("line", [
    "", "START", "START 3", "START 0 0", "START 100 1", "START 1 100", "START 2.5 1", "START 1 2 3", "SETUP 1", "STOP 1", "MANUAL MAYBE", "MOVE 1.5 0",
    "MOVE a b", "DEV", "DEV LED BLUE", "DEV SERVO 90", "DEV SERVO 200 90", "DEV WHEEL 0 0 0",
    "DEV WHEEL 2 0 0 0", "DEV WATCH RANGE", "DEV WATCH FOO ON", "DEV NODES x", "DEV FOO", "JUMP",
    "DEV SPEED", "DEV SPEED 0.1", "DEV SPEED 1.5", "DEV SPEED fast", "DEV SPEED 0.5 0.5",
    "DEV DRIVE", "DEV DRIVE GO", "DEV DRIVE FOLLOW STOP",
    "DEV TUNE", "DEV TUNE TURN", "DEV TUNE KP 1", "DEV TUNE TURN 2", "DEV TUNE CENTER -0.7", "DEV TUNE TURN x",
    "DEV TARGET", "DEV TARGET PINK", "DEV TUNE FORWARD 3", "DEV TUNE SPIN 0.1", "DEV TUNE BLIND 3.5", "DEV TUNE APPROACH 0.1",
    "DEV TUNE CONFIRM 0", "DEV TUNE CONFIRM 2.5", "DEV TUNE LINE_END 2", "DEV TUNE LINE_END 31",
])
def test_parse_invalid(line):
    with pytest.raises(ValueError):
        parse_command(line)


def test_command_key():
    assert Command("DEV", ["LED", "RED"]).key == "DEV LED"
    assert Command("MANUAL", ["OFF"]).key == "MANUAL OFF"
    assert Command("START", ["3", "2"]).key == "START"
    assert command_key("dev led blue") == "DEV LED"
    assert command_key("FOO 1") == "FOO"
    assert command_key("") == ""


@pytest.mark.parametrize("line, state, ok", [
    ("START 3 2", "IDLE", True),
    ("START 3 2", "NAVIGATE", False),
    ("SETUP", "IDLE", True),
    ("SETUP", "NAVIGATE", False),
    ("STOP", "LOAD", True),
    ("RESUME", "ESTOP", True),
    ("RESUME", "IDLE", False),
    ("MANUAL ON", "IDLE", True),
    ("MANUAL OFF", "MANUAL", True),
    ("MOVE 0 0", "IDLE", False),
    ("DEV LED RED", "ESTOP", True),
    ("DEV LED RED", "NAVIGATE", False),
    ("DEV WHEEL 0 0 0 0", "ESTOP", False),
    ("DEV WHEEL 0 0 0 0", "MANUAL", True),
    ("DEV WATCH SYS ON", "NAVIGATE", True),
    ("DEV SPEED 0.6", "NAVIGATE", True),       # 주행 중에도
    ("DEV SPEED 0.6", "ESTOP", True),
    ("DEV DRIVE FOLLOW", "IDLE", True),
    ("DEV DRIVE FOLLOW", "MANUAL", False),       # 수동 조종 중엔 시작 불가
    ("DEV DRIVE FOLLOW", "ESTOP", False),
    ("DEV DRIVE STOP", "ESTOP", True),           # 정지는 언제나
    ("DEV DRIVE STOP", "MANUAL", True),
    ("DEV TUNE TURN 0.5", "NAVIGATE", True),     # 주행 중에도
])
def test_check_allowed(line, state, ok):
    assert (check_allowed(parse_command(line), state) is None) == ok


def test_every_state_accepts_stop_and_ping():
    for s in State:
        assert check_allowed(parse_command("STOP"), s.value) is None
        assert check_allowed(parse_command("PING"), s.value) is None


def test_format_command():
    assert format_command("START", 3, 2) == "START 3 2"
    assert format_command("SETUP") == "SETUP"
    assert format_command("MOVE", 0.5, -0.25) == "MOVE 0.5 -0.25"
    assert format_command("DEV", "WHEEL", 0.3, 0.3, 0.0, 0.0) == "DEV WHEEL 0.3 0.3 0 0"
    with pytest.raises(ValueError):
        format_command("START", 120, 0)


def test_status_roundtrip():
    line = format_status(State.NAVIGATE, Mode.WORK, 1, 5, Cargo.CIRCLE, 3, Target.DROP_A)
    assert line == "STATUS NAVIGATE WORK 1/5 CIRCLE 3 DROP_A"
    r = parse_reply(line)
    assert r == {"type": "STATUS", "state": "NAVIGATE", "mode": "WORK", "done": 1, "total": 5,
                 "cargo": "CIRCLE", "load": 3, "target": "DROP_A"}
    r = parse_reply(format_status("IDLE"))
    assert (r["mode"], r["cargo"], r["load"], r["target"]) == ("NONE", "NONE", 0, "NONE")
    assert parse_reply(format_status("NAVIGATE", "SETUP", 2, 4, target="green"))["target"] == "green"


def test_map_roundtrip():
    line = format_map({"WORK": "blue", "DROP_A": "red"})
    assert line == "MAP WORK=blue DROP_A=red DROP_B=- PARK=-"
    assert parse_reply(line) == {"type": "MAP", "map": {"WORK": "blue", "DROP_A": "red", "DROP_B": None, "PARK": None}}
    for bad in ("MAP WORK=pink", "MAP HOME=red", "MAP WORK"):
        with pytest.raises(ValueError):
            parse_reply(bad)


def test_event_error_ack_nak():
    assert parse_reply(format_event("LOADED", "CIRCLE")) == {"type": "EVENT", "event": "LOADED", "detail": "CIRCLE"}
    assert parse_reply(format_event("PARKED")) == {"type": "EVENT", "event": "PARKED", "detail": ""}
    assert parse_reply(format_error("E_MARKER", "red 갈래 0개"))["detail"] == "red 갈래 0개"
    assert parse_reply(format_ack(parse_command("DEV LED RED"))) == {"type": "ACK", "command": "DEV LED"}
    r = parse_reply(format_nak("DEV WHEEL", "NAVIGATE 상태에서 불가"))
    assert (r["command"], r["reason"]) == ("DEV WHEEL", "NAVIGATE 상태에서 불가")
    r = parse_reply(format_nak("START", "IDLE 아님"))
    assert (r["command"], r["reason"]) == ("START", "IDLE 아님")
    assert parse_reply("PONG") == {"type": "PONG"}


def test_dev_replies():
    assert format_dev("RANGE", 0.234) == "DEV RANGE 0.234"
    assert parse_reply("DEV RANGE 0.234")["distance"] == pytest.approx(0.234)
    r = parse_reply(format_dev("LINE", -0.12, "NONE", "STOP_LINE"))
    assert r["raw_event"] == "STOP_LINE"
    assert parse_reply("DEV LINE 0.1 NONE")["raw_event"] == "NONE"      # 판정 생략 (이전 형식)
    assert parse_reply("DEV LINE 0.1 NONE NONE 87.5")["angle"] == pytest.approx(87.5)
    assert parse_reply("DEV LINE 0.1 NONE NONE -")["angle"] is None     # 각도 못 믿음
    assert parse_reply("DEV LINE 0.1 NONE")["angle"] is None
    r = parse_reply(format_dev("DRIVE", "DONE", "STOP_LINE"))
    assert (r["kind"], r["state"], r["result"]) == ("DRIVE", "DONE", "STOP_LINE")
    assert parse_reply("DEV DRIVE RUNNING")["result"] == ""
    r = parse_reply(format_dev("DRIVE", "RECHECK", "2/3"))
    assert (r["state"], r["result"]) == ("RECHECK", "2/3")
    r = parse_reply(format_dev("LINE", -0.12, "NONE"))
    assert (r["offset"], r["event"]) == (pytest.approx(-0.12), "NONE")
    r = parse_reply(format_dev("SYS", 52.1, 37.0, 41.0))
    assert (r["temp"], r["cpu"], r["mem"]) == (pytest.approx(52.1), 37.0, 41.0)
    assert parse_reply(format_dev("NODES", ["camera", "led_node"]))["nodes"] == ["camera", "led_node"]
    assert parse_reply(format_dev("NODES", []))["nodes"] == []


@pytest.mark.parametrize("line", ["", "HELLO", "STATUS IDLE", "STATUS IDLE NONE 0/0 NONE", "DEV RANGE x", "DEV FOO 1"])
def test_parse_reply_invalid(line):
    with pytest.raises(ValueError):
        parse_reply(line)


def test_dev_marker_reply():
    assert parse_reply(format_dev("MARKER", "red:left:150", "blue:right:90"))["markers"] == [
        ("red", "left", 150.0), ("blue", "right", 90.0)]
    assert parse_reply(format_dev("MARKER"))["markers"] == []


def test_uturn_only_starts_in_idle():
    assert check_allowed(parse_command("DEV DRIVE UTURN"), "IDLE") is None
    assert check_allowed(parse_command("DEV DRIVE UTURN"), "ESTOP")
    assert check_allowed(parse_command("DEV DRIVE STOP"), "ESTOP") is None


@pytest.mark.parametrize("line, ok", [("DEV CAL AUTO", True), ("DEV CAL reset", True), ("DEV CAL", False),
                                      ("DEV CAL PAN", False), ("DEV CAL AUTO 1", False),
                                      ("DEV CAL SIGN 165", True), ("DEV CAL sign 170.5", True), ("DEV CAL SIGN", False),
                                      ("DEV CAL SIGN 200", False), ("DEV CAL SIGN abc", False)])
def test_dev_cal(line, ok):
    if ok:
        assert parse_command(line).args[1] in ("AUTO", "RESET", "SIGN")
    else:
        with pytest.raises(ValueError):
            parse_command(line)


def test_parse_dev_cal_reply():
    r = parse_reply("DEV CAL RUNNING 66.5 87.2")
    assert r == {"type": "DEV", "kind": "CAL", "state": "RUNNING", "detail": "66.5 87.2"}
    assert parse_reply("DEV CAL DONE 66.0")["detail"] == "66.0"


def test_dev_reset_parses_and_is_allowed_everywhere():
    cmd = parse_command("dev reset")
    assert cmd.key == "DEV RESET" and check_allowed(cmd, "ESTOP") is None and check_allowed(cmd, "NAVIGATE") is None
    with pytest.raises(ValueError):
        parse_command("DEV RESET ALL")


def test_parse_dev_log():
    r = parse_reply("DEV LOG STOP 정지선: STOP_LINE 확정 (0.3초)")
    assert r == {"type": "DEV", "kind": "LOG", "category": "STOP", "text": "정지선: STOP_LINE 확정 (0.3초)"}

import pytest

from agv_protocol.messages import (
    Command, check_allowed, command_key, format_ack, format_command, format_dev, format_error,
    format_event, format_nak, format_status, parse_command, parse_reply,
)
from agv_protocol.types import Cargo, State, Target


@pytest.mark.parametrize("line, kind, args", [
    ("START 3", "START", ["3"]),
    ("stop", "STOP", []),
    ("MANUAL on", "MANUAL", ["ON"]),
    ("MOVE 0.5 -0.2", "MOVE", ["0.5", "-0.2"]),
    ("DEV led RED", "DEV", ["LED", "RED"]),
    ("DEV SERVO 90 60", "DEV", ["SERVO", "90", "60"]),
    ("DEV WHEEL 0.3 0.3 0 -1", "DEV", ["WHEEL", "0.3", "0.3", "0", "-1"]),
    ("DEV WATCH ALL ON", "DEV", ["WATCH", "ALL", "ON"]),
    ("DEV NODES", "DEV", ["NODES"]),
])
def test_parse_valid(line, kind, args):
    cmd = parse_command(line)
    assert (cmd.kind, cmd.args) == (kind, args)


@pytest.mark.parametrize("line", [
    "", "START", "START 0", "START 10", "START 2.5", "STOP 1", "MANUAL MAYBE", "MOVE 1.5 0",
    "MOVE a b", "DEV", "DEV LED BLUE", "DEV SERVO 90", "DEV SERVO 200 90", "DEV WHEEL 0 0 0",
    "DEV WHEEL 2 0 0 0", "DEV WATCH RANGE", "DEV WATCH FOO ON", "DEV NODES x", "DEV FOO", "JUMP",
])
def test_parse_invalid(line):
    with pytest.raises(ValueError):
        parse_command(line)


def test_command_key():
    assert Command("DEV", ["LED", "RED"]).key == "DEV LED"
    assert Command("MANUAL", ["OFF"]).key == "MANUAL OFF"
    assert Command("START", ["3"]).key == "START"
    assert command_key("dev led blue") == "DEV LED"
    assert command_key("FOO 1") == "FOO"
    assert command_key("") == ""


@pytest.mark.parametrize("line, state, ok", [
    ("START 3", "IDLE", True),
    ("START 3", "NAVIGATE", False),
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
])
def test_check_allowed(line, state, ok):
    assert (check_allowed(parse_command(line), state) is None) == ok


def test_every_state_accepts_stop_and_ping():
    for s in State:
        assert check_allowed(parse_command("STOP"), s.value) is None
        assert check_allowed(parse_command("PING"), s.value) is None


def test_format_command():
    assert format_command("START", 3) == "START 3"
    assert format_command("MOVE", 0.5, -0.25) == "MOVE 0.5 -0.25"
    assert format_command("DEV", "WHEEL", 0.3, 0.3, 0.0, 0.0) == "DEV WHEEL 0.3 0.3 0 0"
    with pytest.raises(ValueError):
        format_command("START", 12)


def test_status_roundtrip():
    line = format_status(State.NAVIGATE, 1, 3, Cargo.CIRCLE, Target.DROP_A)
    assert line == "STATUS NAVIGATE 1/3 CIRCLE DROP_A"
    r = parse_reply(line)
    assert r == {"type": "STATUS", "state": "NAVIGATE", "done": 1, "total": 3,
                 "cargo": "CIRCLE", "target": "DROP_A"}
    assert parse_reply(format_status("IDLE", 0, 0))["cargo"] == "NONE"


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
    r = parse_reply(format_dev("LINE", -0.12, "NONE"))
    assert (r["offset"], r["event"]) == (pytest.approx(-0.12), "NONE")
    r = parse_reply(format_dev("SYS", 52.1, 37.0, 41.0))
    assert (r["temp"], r["cpu"], r["mem"]) == (pytest.approx(52.1), 37.0, 41.0)
    assert parse_reply(format_dev("NODES", ["camera", "led_node"]))["nodes"] == ["camera", "led_node"]
    assert parse_reply(format_dev("NODES", []))["nodes"] == []


@pytest.mark.parametrize("line", ["", "HELLO", "STATUS IDLE", "DEV RANGE x", "DEV FOO 1"])
def test_parse_reply_invalid(line):
    with pytest.raises(ValueError):
        parse_reply(line)

"""DriveController·PidController — 가짜 라인 상태 순서로 시험 (Pi 없이)."""
import pytest

from agv_control.drive import DriveConfig, DriveController, LineInput, PidController
from agv_protocol.types import DriveCommand, LineEvent

DT = 0.05


def follow():
    d = DriveController(DriveConfig())
    d.start(DriveCommand.FOLLOW_TO_EVENT)
    return d


def test_pid_direction_and_size():
    pid = PidController(kp=1.0, kd=0.0)
    assert pid.update(0.5, DT) == pytest.approx(0.5)
    assert pid.update(-0.2, DT) == pytest.approx(-0.2)


def test_pid_derivative_damps_change():
    pid = PidController(kp=0.0, kd=0.1)
    pid.update(0.0, DT)
    assert pid.update(0.5, DT) == pytest.approx(1.0)        # 0.1 × 0.5/0.05


@pytest.mark.parametrize("offset, sign", [(0.4, -1), (-0.4, 1)])
def test_turns_toward_line(offset, sign):
    """라인이 오른쪽(+)이면 오른쪽 회전(z < 0), 왼쪽이면 왼쪽 회전(z > 0)."""
    out = follow().step(LineInput(offset, True), DT)
    assert out.angular * sign > 0 and out.linear > 0 and not out.done


def test_straight_when_centered():
    out = follow().step(LineInput(0.0, True), DT)
    assert out.angular == 0 and out.linear == pytest.approx(DriveConfig().speed)


def test_slows_down_with_large_offset_but_not_below_min():
    c = DriveConfig()
    near = follow().step(LineInput(0.1, True), DT).linear
    far = follow().step(LineInput(0.9, True), DT).linear
    assert far < near and far >= c.min_speed


def test_angular_is_limited():
    out = follow().step(LineInput(1.0, True), DT)
    assert abs(out.angular) <= DriveConfig().max_angular


def test_slows_when_event_appears():
    out = follow().step(LineInput(0.0, True, LineEvent.NONE, LineEvent.STOP_LINE), DT)
    assert out.linear == pytest.approx(DriveConfig().event_speed)


@pytest.mark.parametrize("event", [LineEvent.STOP_LINE, LineEvent.CORNER_LEFT, LineEvent.CORNER_RIGHT, LineEvent.LINE_END])
def test_stops_on_confirmed_event(event):
    d = follow()
    out = d.step(LineInput(0.0, True, event, event), DT)
    assert out.done and out.result == event.value and (out.linear, out.angular) == (0, 0)
    assert not d.active


def test_junction_event_does_not_stop():
    """분기점은 마커 인식 담당 — 라인 쪽 JUNCTION이 와도 계속 주행."""
    out = follow().step(LineInput(0.0, True, LineEvent.JUNCTION_T, LineEvent.JUNCTION_T), DT)
    assert not out.done and out.linear > 0


def test_keeps_last_output_between_frames():
    d = follow()
    first = d.step(LineInput(0.3, True), DT)
    assert d.step(None, DT) == first


def test_stops_when_line_state_stops_arriving():
    d = follow()
    d.step(LineInput(0.0, True), DT)
    out = None
    for _ in range(11):                            # 0.55초
        out = d.step(None, DT)
    assert out.done and out.result == "E_LINE" and out.linear == 0


def test_stop_and_cancel():
    d = DriveController()
    d.start(DriveCommand.STOP)
    assert d.step(LineInput(0.0, True), DT).result == "STOPPED"
    d = follow()
    assert d.cancel().result == "STOPPED" and not d.active


def test_unsupported_command_fails_safely():
    d = DriveController()
    d.start(DriveCommand.UTURN)
    out = d.step(LineInput(0.0, True), DT)
    assert out.done and out.result == "E_UNSUPPORTED_UTURN" and out.linear == 0


def test_set_speed_applies_while_driving():
    d = follow()
    assert d.step(LineInput(0.0, True), DT).linear == pytest.approx(0.6)
    d.set_speed(0.8)
    assert d.step(LineInput(0.0, True), DT).linear == pytest.approx(0.8)


def test_set_speed_lower_than_floor_lowers_floor():
    """하한(0.45)보다 낮게 정하면 그 속도로 — 오프셋이 커도 정한 값보다 빨라지지 않음."""
    d = follow()
    d.set_speed(0.3)
    assert d.step(LineInput(0.0, True), DT).linear == pytest.approx(0.3)
    assert d.step(LineInput(0.9, True), DT).linear <= 0.3
    assert d.step(LineInput(0.0, True, LineEvent.NONE, LineEvent.STOP_LINE), DT).linear <= 0.3
    d.set_speed(0.7)                                  # 다시 올리면 원래 하한 복구
    assert d.cfg.min_speed == pytest.approx(0.45) and d.cfg.event_speed == pytest.approx(0.45)


def test_heading_error_deadband():
    from agv_control.drive import heading_error
    assert heading_error(90, 5) == 0 and heading_error(86, 5) == 0 and heading_error(94.9, 5) == 0
    assert heading_error(80, 5) == pytest.approx(5)          # 위쪽이 오른쪽 → +
    assert heading_error(100, 5) == pytest.approx(-5)


@pytest.mark.parametrize("angle, sign", [(70, -1), (110, 1)])
def test_turns_to_align_with_tilted_line(angle, sign):
    """오프셋 0이어도 라인이 기울어 있으면 라인 방향으로 회전 (70° = 위쪽이 오른쪽 → 오른쪽 회전 z<0)."""
    out = follow().step(LineInput(0.0, True, angle=angle, angle_valid=True), DT)
    assert out.angular * sign > 0


@pytest.mark.parametrize("angle", [85.5, 90, 94.5])
def test_no_heading_correction_inside_85_95(angle):
    out = follow().step(LineInput(0.0, True, angle=angle, angle_valid=True), DT)
    assert out.angular == 0


def test_invalid_angle_is_ignored():
    out = follow().step(LineInput(0.0, True, angle=60, angle_valid=False), DT)
    assert out.angular == 0


def test_heading_adds_to_offset_correction():
    only_offset = follow().step(LineInput(0.2, True), DT).angular
    both = follow().step(LineInput(0.2, True, angle=75, angle_valid=True), DT).angular
    assert both < only_offset < 0                             # 둘 다 오른쪽 → 더 크게 오른쪽


def test_idle_outputs_zero():
    assert DriveController().step(LineInput(0.5, True), DT).linear == 0

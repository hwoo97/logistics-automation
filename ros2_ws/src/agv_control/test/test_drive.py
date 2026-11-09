"""DriveController·PidController — 가짜 라인 상태 순서로 시험 (Pi 없이)."""
import pytest

from agv_control.drive import DriveConfig, DriveController, LineInput, MarkerInput, PidController
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


def test_slows_when_line_lost_for_a_moment():
    out = follow().step(LineInput(0.0, False), DT)
    assert out.linear == pytest.approx(DriveConfig().event_speed)


LOST = LineInput(0.0, False, LineEvent.LINE_END, LineEvent.LINE_END)
FOUND = LineInput(0.1, True)


def test_line_end_stops_and_rechecks_then_finishes():
    """라인 끝 → 정지 후 3번(1.5초) 다시 봄 → 계속 없으면 진짜 라인 끝으로 끝남."""
    d = follow()
    first = d.step(LOST, DT)
    assert not first.done and (first.linear, first.angular) == (0, 0) and d.phase == "RECHECK 1/3"
    outs, phases = [], set()
    for _ in range(60):
        out = d.step(LOST, DT)
        outs.append(out)
        if out.done:
            break
        phases.add(d.phase)
    assert outs[-1].done and outs[-1].result == "LINE_END" and not d.active
    assert all(o.linear == 0 for o in outs)                  # 재확인 중엔 정지
    assert phases == {"RECHECK 1/3", "RECHECK 2/3", "RECHECK 3/3"}
    assert len(outs) == pytest.approx(30, abs=1)             # 3 × 0.5초 ÷ 0.05초


def test_line_end_resumes_when_line_seen_again():
    """주행 중 잠깐 놓쳐 라인 끝이 됐어도 멈춘 뒤 3프레임 연속 보이면 자동 재출발 (2026-11-02 S1: 앱에서 다시 누르면 달리던 경우)."""
    d = follow()
    d.step(LOST, DT)
    assert d.step(FOUND, DT).linear == 0 and d.step(FOUND, DT).linear == 0
    d.step(FOUND, DT)                                        # 3프레임째 — 재확인 끝
    assert d.phase == "FOLLOW" and d.active
    out = d.step(FOUND, DT)
    assert out.linear > 0 and not out.done


def test_recheck_needs_consecutive_frames():
    d = follow()
    d.step(LOST, DT)
    for line in (FOUND, FOUND, LOST, FOUND, FOUND):          # 중간에 끊기면 처음부터
        d.step(line, DT)
    assert d.phase.startswith("RECHECK")


def test_recheck_zero_finishes_immediately():
    c = DriveConfig(recheck_tries=0)
    d = DriveController(c)
    d.start(DriveCommand.FOLLOW_TO_EVENT)
    out = d.step(LOST, DT)
    assert out.done and out.result == "LINE_END"


def test_recheck_stops_if_line_state_stops_arriving():
    d = follow()
    d.step(LOST, DT)
    out = None
    for _ in range(11):
        out = d.step(None, DT)
    assert out.done and out.result == "E_LINE"


def test_cancel_during_recheck():
    d = follow()
    d.step(LOST, DT)
    assert d.cancel().result == "STOPPED" and not d.active and d.phase == "FOLLOW"


RAW_STOP = LineInput(0.0, True, LineEvent.NONE, LineEvent.STOP_LINE)
CONFIRMED_STOP = LineInput(0.0, True, LineEvent.STOP_LINE, LineEvent.STOP_LINE)
CLEAR = LineInput(0.0, True)


def test_merge_stop_line():
    from agv_control.drive import merge_stop_line
    base = LineInput(0.2, True, LineEvent.NONE, LineEvent.NONE, 88.0, True)
    assert merge_stop_line(base, False, False) == base
    m = merge_stop_line(base, True, False)
    assert (m.raw_event, m.event, m.offset, m.angle) == (LineEvent.STOP_LINE, LineEvent.NONE, 0.2, 88.0)
    m = merge_stop_line(base, True, True)
    assert (m.raw_event, m.event) == (LineEvent.STOP_LINE, LineEvent.STOP_LINE)


def test_halts_on_first_raw_stop_line():
    """판정 첫 프레임에 바로 정지 — 달리면서 확인 프레임을 채우면 지나침 (2026-11-02 S1)."""
    d = follow()
    assert d.step(CLEAR, DT).linear > 0
    out = d.step(RAW_STOP, DT)
    assert (out.linear, out.angular) == (0, 0) and not out.done and d.phase == "VERIFY STOP_LINE"


def test_verified_stop_line_finishes_while_stopped():
    d = follow()
    d.step(RAW_STOP, DT)
    assert d.step(RAW_STOP, DT).linear == 0 and d.step(None, DT).linear == 0
    out = d.step(CONFIRMED_STOP, DT)
    assert out.done and out.result == "STOP_LINE" and not d.active


def test_false_stop_line_resumes():
    """판정이 사라지면 오판정 — 다시 출발, 잠시 같은 판정으로 안 멈춤."""
    d = follow()
    d.step(RAW_STOP, DT)
    for _ in range(5):
        assert d.step(CLEAR, DT).linear == 0
    assert d.phase == "FOLLOW" and d.active
    assert d.step(CLEAR, DT).linear > 0
    assert d.step(RAW_STOP, DT).linear > 0                   # 재정지 금지 시간
    for _ in range(11):
        d.step(CLEAR, DT)
    assert d.step(RAW_STOP, DT).linear == 0                  # 0.5초 뒤엔 다시 멈춤


def test_flickering_judgement_times_out():
    d = follow()
    d.step(RAW_STOP, DT)
    for i in range(45):
        d.step(RAW_STOP if i % 2 else CLEAR, DT)
    assert d.phase == "FOLLOW" and d.active                  # 2초 뒤 다시 출발, 멈춘 채 갇히지 않음


def test_confirmed_during_cooldown_still_stops():
    d = follow()
    d.step(RAW_STOP, DT)
    for _ in range(5):
        d.step(CLEAR, DT)
    out = d.step(CONFIRMED_STOP, DT)
    assert out.done and out.result == "STOP_LINE"


def test_corner_also_verified_while_stopped():
    d = follow()
    out = d.step(LineInput(0.0, True, LineEvent.NONE, LineEvent.CORNER_LEFT), DT)
    assert out.linear == 0 and d.phase == "VERIFY CORNER_LEFT"


def test_cancel_during_verify():
    d = follow()
    d.step(RAW_STOP, DT)
    assert d.cancel().result == "STOPPED" and d.phase == "FOLLOW"


def test_verify_stops_if_line_state_stops_arriving():
    d = follow()
    d.step(RAW_STOP, DT)
    out = None
    for _ in range(11):
        out = d.step(None, DT)
    assert out.done and out.result == "E_LINE"


@pytest.mark.parametrize("event", [LineEvent.STOP_LINE, LineEvent.CORNER_LEFT, LineEvent.CORNER_RIGHT])
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
    d.start(DriveCommand.STRAIGHT)
    out = d.step(LineInput(0.0, True), DT)
    assert out.done and out.result == "E_UNSUPPORTED_STRAIGHT" and out.linear == 0


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


def test_center_offset_shifts_target():
    """카메라가 왼쪽에 달려 기준 위치 −0.1 — 라인이 −0.1에 있으면 직진, 가운데(0)에 있으면 오른쪽으로."""
    d = follow()
    d.set_tune("CENTER", -0.1)
    assert d.step(LineInput(-0.1, True), DT).angular == 0
    d = follow()
    d.set_tune("CENTER", -0.1)
    assert d.step(LineInput(0.0, True), DT).angular < 0


def test_turn_scale_scales_rotation():
    full = follow().step(LineInput(0.2, True, angle=80, angle_valid=True), DT).angular
    d = follow()
    d.set_tune("TURN", 0.5)
    half = d.step(LineInput(0.2, True, angle=80, angle_valid=True), DT).angular
    assert half == pytest.approx(full / 2, abs=1e-3)


def test_turn_scale_does_not_change_speed():
    d = follow()
    d.set_tune("TURN", 0.3)
    assert d.step(LineInput(0.0, True), DT).linear == pytest.approx(DriveConfig().speed)


def test_unknown_tune_key():
    with pytest.raises(ValueError):
        follow().set_tune("KP", 1.0)


def test_idle_outputs_zero():
    assert DriveController().step(LineInput(0.5, True), DT).linear == 0


# ── 마커 ─────────────────────────────────────────────────

AHEAD = LineInput(0.0, True, angle=90.0, angle_valid=True)
GONE = LineInput(0.0, False)
BAR = LineInput(0.0, True, angle=2.0, angle_valid=True)     # T자 가로선


def marker_drive(target="red", **cfg):
    d = DriveController(DriveConfig(**cfg))
    d.set_target(target)
    d.start(DriveCommand.FOLLOW_TO_EVENT)
    return d


def see(*sides, y=130.0, color="red"):
    return [MarkerInput(color, s, y) for s in sides]


def branches(left=False, right=False):
    """분기점이 보이는 라인 — 갈래 쪽 (line_node branch_left/right)."""
    return LineInput(0.0, True, angle=90.0, angle_valid=True, branch_left=left, branch_right=right)


def check(d, *sides, y=130.0, line=None):
    """멈춘 뒤 확인 시간이 끝날 때까지 같은 마커를 본다. 라인은 기본으로 스티커 쪽에 갈래가 있는 분기점."""
    line = line or branches("left" in sides, "right" in sides)
    for _ in range(40):
        d.step(line, DT, see(*sides, y=y))
        if not d.phase.startswith("MARKER"):
            break


def run_forward(d):
    while d.turn_stage == "FORWARD":
        d.step(AHEAD, DT)


def test_markers_ignored_without_target():
    d = marker_drive(target=None)
    assert d.step(AHEAD, DT, see("left", y=200.0)).linear > 0 and d.phase == "FOLLOW"


def test_other_color_is_ignored():
    d = marker_drive("red")
    assert d.step(AHEAD, DT, see("left", y=200.0, color="blue")).linear > 0 and d.phase == "FOLLOW"


def test_far_marker_slows_down_near_marker_stops():
    c = DriveConfig()
    d = marker_drive()
    assert d.step(AHEAD, DT, see("left", y=30.0)).linear == pytest.approx(c.speed)      # 멀면 그대로
    assert d.step(AHEAD, DT, see("left", y=80.0)).linear == pytest.approx(c.event_speed)
    out = d.step(AHEAD, DT, see("left", y=125.0))
    assert out.linear == 0 and out.angular == 0 and d.phase == "MARKER red"


def test_slowdown_ends_when_marker_is_gone():
    c = DriveConfig()
    d = marker_drive()
    d.step(AHEAD, DT, see("left", y=80.0))
    assert d.step(AHEAD, DT, []).linear == pytest.approx(c.speed)


@pytest.mark.parametrize("side, phase, sign", [("left", "TURN LEFT", 1), ("right", "TURN RIGHT", -1)])
def test_one_side_turns_that_way(side, phase, sign):
    d = marker_drive()
    d.step(AHEAD, DT, see(side))
    check(d, side)
    assert d.phase == phase
    assert d.step(AHEAD, DT).linear > 0                     # 먼저 전진
    run_forward(d)
    out = d.step(AHEAD, DT)
    assert out.linear == 0 and out.angular * sign > 0


def test_both_sides_in_one_frame_pass_without_stopping():
    c = DriveConfig()
    d = marker_drive()
    out = d.step(AHEAD, DT, see("left", "right"))
    assert out.linear > 0 and d.phase == "FOLLOW"
    assert d.marker_ignore == pytest.approx(c.marker_ignore_s)
    assert d.step(AHEAD, DT, see("left")).linear > 0 and d.phase == "FOLLOW"      # 지나가는 동안 다시 안 멈춤


def test_both_sides_found_while_stopped_goes_straight():
    d = marker_drive()
    d.step(AHEAD, DT, see("left"))
    check(d, "left", "right")
    assert d.phase == "FOLLOW" and d.step(AHEAD, DT).linear > 0


def test_marker_gone_while_checking_resumes():
    """한 프레임 오인식 — 멈춰서 보니 없으면 다시 출발."""
    d = marker_drive()
    d.step(AHEAD, DT, see("left"))
    for _ in range(40):
        d.step(AHEAD, DT, [])
        if d.phase == "FOLLOW":
            break
    assert d.phase == "FOLLOW" and d.step(AHEAD, DT, see("left")).linear > 0


def test_check_times_out_without_frames():
    d = marker_drive()
    d.step(AHEAD, DT, see("left"))
    for _ in range(int(DriveConfig().marker_check_timeout_s / DT) + 1):
        d.step(AHEAD, DT)
    assert d.phase == "FOLLOW"


def test_check_ignores_frames_before_settle():
    """멈추는 흔들림 동안 본 프레임은 세지 않음."""
    d = marker_drive(marker_settle_s=0.5)
    d.step(AHEAD, DT, see("left"))
    for _ in range(5):
        d.step(AHEAD, DT, see("right"))
    assert d.check_counts == [0, 0, 0]


def test_check_ignores_far_marker():
    """확인 중 멀리(건너편) 보이는 같은 색은 세지 않음 → 왼쪽만."""
    d = marker_drive()
    d.step(AHEAD, DT, see("left"))
    for _ in range(40):
        d.step(branches(left=True), DT, see("left") + see("right", y=50.0))
        if not d.phase.startswith("MARKER"):
            break
    assert d.phase == "TURN LEFT"


# ── 스티커 쪽에 갈래가 없으면 직진 (2026-11-06 실차: 직진·좌회전 갈래뿐인 곳, 스티커 오른쪽 → 유턴처럼 돎) ──

def test_right_sticker_without_right_branch_goes_straight():
    d = marker_drive()
    d.step(AHEAD, DT, see("right"))
    check(d, "right", line=branches(left=True))              # 왼쪽 갈래만 있음
    assert d.phase == "FOLLOW" and d.turn_dir == 0
    assert "갈래 없음" in d.decision
    out = d.step(AHEAD, DT, see("right"))                    # 같은 스티커로 다시 멈추지 않고 전진
    assert out.linear > 0 and d.phase == "FOLLOW"


@pytest.mark.parametrize("side", ["left", "right"])
def test_sticker_without_any_branch_goes_straight(side):
    d = marker_drive()
    d.step(AHEAD, DT, see(side))
    check(d, side, line=branches())
    assert d.phase == "FOLLOW" and d.turn_dir == 0


def test_branch_seen_in_minority_of_frames_goes_straight():
    d = marker_drive()
    d.step(AHEAD, DT, see("left"))
    lines = iter([branches(left=True)] + [branches()] * 40)
    for _ in range(40):
        d.step(next(lines), DT, see("left"))
        if not d.phase.startswith("MARKER"):
            break
    assert d.phase == "FOLLOW"


def test_branch_check_off_turns_on_sticker_only():
    d = marker_drive(marker_branch_check=False)
    d.step(AHEAD, DT, see("right"))
    check(d, "right", line=branches(left=True))
    assert d.phase == "TURN RIGHT"


def test_without_line_frames_turns_on_sticker_only():
    """확인 중 라인 상태를 하나도 못 받으면 갈래를 모름 → 스티커대로 회전 (이전 동작)."""
    d = marker_drive()
    d.step(AHEAD, DT, see("left"))
    for _ in range(40):
        d.step(None, DT, see("left"))
        if not d.phase.startswith("MARKER"):
            break
    assert d.phase == "TURN LEFT"


def test_decision_counter_increases_each_time():
    d = marker_drive()
    d.step(AHEAD, DT, see("left"))
    check(d, "left")
    assert d.decision == "LEFT" and d.decision_no == 1


def turning(side="left", **cfg):
    """회전 시작 직후. 기본은 라인 무시 없음(turn_blind_s=0) — 기존 라인 벗어나기(LEAVE) 방식 시험."""
    cfg.setdefault("turn_blind_s", 0.0)
    d = marker_drive(**cfg)
    d.step(AHEAD, DT, see(side))
    check(d, side)
    run_forward(d)
    return d


def test_turn_does_not_end_on_old_line():
    """회전 시작 때 보이는 기존 라인으로는 끝나지 않음."""
    d = turning()
    for _ in range(20):
        d.step(AHEAD, DT)
    assert d.phase == "TURN LEFT"


def test_turn_does_not_end_on_t_bar():
    """T자 가로선(각도가 눕음)은 새 라인이 아님."""
    d = turning()
    for _ in range(40):
        d.step(BAR, DT)
    assert d.phase == "TURN LEFT"


def test_turn_does_not_end_on_line_at_screen_edge():
    d = turning()
    d.step(GONE, DT), d.step(GONE, DT)
    edge = LineInput(-0.9, True, angle=90.0, angle_valid=True)
    for _ in range(20):
        d.step(edge, DT)
    assert d.phase == "TURN LEFT"


def test_turn_ends_on_new_line():
    d = turning()
    for _ in range(12):
        d.step(GONE, DT)                                    # 기존 라인 벗어남 + 최소 시간 지남
    d.step(AHEAD, DT), d.step(AHEAD, DT)
    assert d.phase == "TURN LEFT"                           # 아직 2프레임
    d.step(AHEAD, DT)
    assert d.phase == "FOLLOW"
    assert d.step(AHEAD, DT).linear > 0


@pytest.mark.parametrize("side, sign", [("right", -1), ("left", 1)])
def test_turn_new_line_enters_before_old_line_leaves(side, sign):
    """광각 — 기존 라인이 화면에 남은 채 새 라인이 들어와 오프셋이 도는 쪽으로 튐. 끊김 없이도 새 라인에서 끝나야 함."""
    d = turning(side)
    for offset in (0.0, 0.1, 0.2, 0.3):                     # 기존 라인은 도는 쪽 반대로 밀림 (오른쪽으로 돌면 왼쪽으로)
        d.step(LineInput(sign * offset, True, angle=90.0, angle_valid=True), DT)
    for _ in range(8):
        d.step(LineInput(-sign * 0.1, True, angle=90.0, angle_valid=True), DT)
        if d.phase == "FOLLOW":
            break
    assert d.phase == "FOLLOW"


def test_turn_old_line_sliding_away_is_not_a_jump():
    """기존 라인이 조금씩 밀리는 동안(프레임당 0.1)은 새 라인으로 보지 않음."""
    d = turning("right")
    for i in range(4):
        d.step(LineInput(-0.1 * i, True, angle=90.0, angle_valid=True), DT)
    assert d.turn_stage == "LEAVE" and d.phase == "TURN RIGHT"


def test_turn_accepts_tilted_new_line():
    """회전 각도와 상관없음 — 비스듬한 라인도 허용 범위면 끝, 이후 각도 보정이 맞춘다."""
    d = turning()
    for _ in range(12):
        d.step(GONE, DT)
    tilted = LineInput(-0.3, True, angle=120.0, angle_valid=True)
    for _ in range(3):
        d.step(tilted, DT)
    assert d.phase == "FOLLOW"
    assert d.step(tilted, DT).angular > 0                   # 왼쪽 라인·왼쪽 기움 → 왼쪽으로


def test_turn_needs_consecutive_frames():
    d = turning()
    for _ in range(8):
        d.step(GONE, DT)
    for _ in range(10):
        d.step(AHEAD, DT), d.step(GONE, DT)
    assert d.phase == "TURN LEFT"


def test_turn_timeout_is_e_line():
    c = DriveConfig()
    d = turning()
    out = None
    for _ in range(int(c.turn_timeout_s / DT) + 1):
        out = d.step(GONE, DT)
    assert out.done and out.result == "E_LINE" and out.linear == 0 and out.angular == 0
    assert not d.active and not d.turn_dir



BLIND = DriveConfig().turn_blind_s


def blind_turning(side="left"):
    return turning(side, turn_blind_s=BLIND)


def test_blind_default_is_two_seconds():
    assert BLIND == 2.0


@pytest.mark.parametrize("side, sign", [("left", 1), ("right", -1)])
def test_blind_turn_spins_without_looking(side, sign):
    """회전 시작 후 2초는 라인이 앞에 보여도(기존 라인·옆 갈래) 끝나지 않고 그쪽으로 계속 돔."""
    d = blind_turning(side)
    for _ in range(int(BLIND / DT) - 2):
        out = d.step(AHEAD, DT)
        assert out.linear == 0 and out.angular * sign > 0
    assert d.turn_stage == "BLIND" and d.phase.startswith("TURN")


def test_blind_turn_ignores_line_lost_and_jumps():
    """무시 시간 중에는 라인이 사라지거나 튀어도 단계가 바뀌지 않음."""
    d = blind_turning()
    for i in range(int(BLIND / DT) - 2):
        d.step(GONE if i % 2 else LineInput(0.4, True, angle=90.0, angle_valid=True), DT)
    assert d.turn_stage == "BLIND"


def test_blind_turn_ends_on_line_right_after_blind():
    """무시 시간이 끝나면 기존 라인 벗어나기 없이 바로 새 라인 찾기 — 그때 앞에 라인이 있으면 끝."""
    d = blind_turning()
    for _ in range(int(BLIND / DT) + 1):
        d.step(GONE, DT)
    assert d.turn_stage == "SEEK"
    for _ in range(20):
        d.step(AHEAD, DT)
        if d.phase == "FOLLOW":
            break
    assert d.phase == "FOLLOW"


def test_blind_turn_timeout_counts_after_blind():
    c = DriveConfig()
    d = blind_turning()
    for _ in range(int(BLIND / DT) + 1):
        assert not d.step(GONE, DT).done
    out = None
    for _ in range(int(c.turn_timeout_s / DT) - 2):
        out = d.step(GONE, DT)
    assert not out.done and d.phase == "TURN LEFT"           # 무시 시간 + 거의 4초 — 아직 찾는 중
    for _ in range(4):
        out = d.step(GONE, DT)
        if out.done:
            break
    assert out.done and out.result == "E_LINE"


def test_blind_tune_from_app():
    d = blind_turning()
    d.set_tune("BLIND", 0.0)                                  # 회전 중 0으로 바꾸면 다음 프레임부터 찾기
    d.step(GONE, DT)
    assert d.turn_stage == "SEEK"


def test_uturn_has_no_blind():
    """유턴은 라인 무시를 쓰지 않음 (uturn_min_s로 따로)."""
    d = DriveController(DriveConfig())
    d.start(DriveCommand.UTURN)
    d.step(AHEAD, DT)
    assert d.turn_stage == "LEAVE"

def finished_turn():
    d = turning()
    for _ in range(8):
        d.step(GONE, DT)
    for _ in range(3):
        d.step(AHEAD, DT)
    assert d.phase == "FOLLOW"
    return d


@pytest.mark.parametrize("event", [LineEvent.STOP_LINE, LineEvent.CORNER_LEFT, LineEvent.LINE_END])
def test_events_ignored_right_after_turn(event):
    """회전 때 잡힌 분기점 가로선이 확정돼 와도 멈추지 않음."""
    d = finished_turn()
    out = d.step(LineInput(0.0, True, event, event), DT)
    assert out.linear > 0 and not out.done and d.phase == "FOLLOW"


def test_events_work_again_after_settle():
    d = finished_turn()
    for _ in range(int(DriveConfig().turn_settle_s / DT) + 1):
        d.step(AHEAD, DT)
    out = d.step(LineInput(0.0, True, LineEvent.STOP_LINE, LineEvent.STOP_LINE), DT)
    assert out.done and out.result == "STOP_LINE"


def test_markers_ignored_right_after_turn_then_work():
    d = finished_turn()
    assert d.step(AHEAD, DT, see("left")).linear > 0 and d.phase == "FOLLOW"
    for _ in range(int(DriveConfig().marker_ignore_s / DT) + 1):
        d.step(AHEAD, DT)
    d.step(AHEAD, DT, see("left"))
    assert d.phase == "MARKER red"


def test_cancel_during_turn_stops():
    d = turning()
    out = d.cancel()
    assert out.done and out.linear == 0 and not d.turn_dir and d.phase == "FOLLOW"


def test_line_timeout_during_turn():
    d = turning()
    out = None
    for _ in range(int(DriveConfig().line_timeout_s / DT) + 2):
        out = d.step(None, DT)
        if out.done:
            break
    assert out.done and out.result == "E_LINE" and not d.turn_dir


def test_tune_forward_and_spin():
    d = marker_drive()
    d.set_tune("FORWARD", 0.8), d.set_tune("SPIN", 0.5)
    assert d.cfg.turn_forward_s == 0.8 and d.cfg.turn_speed == 0.5


# ── 유턴 ─────────────────────────────────────────────────

def uturn(**cfg):
    d = DriveController(DriveConfig(**cfg))
    d.start(DriveCommand.UTURN)
    return d


def test_uturn_spins_clockwise_in_place_without_forward():
    """기본 유턴 = 시계 방향(오른쪽, 각속도 음수) — 2026-11-09 좌회전 약한 하드웨어."""
    d = uturn()
    out = d.step(AHEAD, DT)
    assert out.linear == 0 and out.angular == pytest.approx(-DriveConfig().turn_speed) and d.phase == "UTURN"


def test_uturn_dir_left_restores_previous():
    d = uturn(uturn_dir=1)
    assert d.step(AHEAD, DT).angular == pytest.approx(DriveConfig().turn_speed)


def test_uturn_follows_spin_tune():
    d = uturn()
    d.set_tune("SPIN", 0.4)
    assert d.step(AHEAD, DT).angular == pytest.approx(-0.4)


@pytest.mark.parametrize("udir", [-1, 1])
def test_uturn_align_without_line_turns_uturn_way(udir):
    """맞추기 중 라인을 한 번도 못 봤으면 유턴하던 쪽으로 계속."""
    d = uturn(uturn_dir=udir)
    d.turn_stage, d.turn_offset = "ALIGN", None
    assert d.step(GONE, DT).angular == pytest.approx(udir * DriveConfig().align_min)


def test_uturn_does_not_end_on_old_line():
    d = uturn()
    for _ in range(100):
        assert not d.step(AHEAD, DT).done
    assert d.phase == "UTURN"


def test_uturn_ignores_line_seen_too_early():
    """시작 직후 잠깐 보이는 정지선·옆 라인에서는 끝나지 않음."""
    d = uturn()
    for _ in range(3):
        d.step(GONE, DT)
    for _ in range(int(DriveConfig().uturn_min_s / DT) - 8):
        assert not d.step(AHEAD, DT).done


def test_uturn_ends_when_line_found_again():
    d = uturn()
    for _ in range(int(DriveConfig().uturn_min_s / DT) + 1):
        d.step(GONE, DT)
    d.step(AHEAD, DT), d.step(AHEAD, DT)
    assert not d.step(AHEAD, DT).done and d.turn_stage == "ALIGN"   # 찾음 → 맞추기 (이미 가운데)
    out = d.step(AHEAD, DT)
    assert out.done and out.result == "UTURN_DONE" and out.linear == 0 and out.angular == 0
    assert not d.active and d.phase == "FOLLOW"


def test_uturn_without_align_ends_on_found():
    d = uturn(uturn_align=False)
    for _ in range(int(DriveConfig().uturn_min_s / DT) + 1):
        d.step(GONE, DT)
    d.step(AHEAD, DT), d.step(AHEAD, DT)
    out = d.step(AHEAD, DT)
    assert out.done and out.result == "UTURN_DONE"


def at(offset):
    return LineInput(offset, True, angle=90.0, angle_valid=True)


def aligning(first=0.0, **cfg):
    """유턴에서 라인을 찾아 맞추기 단계에 들어간 상태. first = 찾은 순간 라인 위치."""
    d = uturn(**cfg)
    for _ in range(int(DriveConfig().uturn_min_s / DT) + 1):
        d.step(GONE, DT)
    for _ in range(DriveConfig().turn_seen_frames - 1):
        d.step(at(first), DT)
    d.step(at(first), DT)
    return d


@pytest.mark.parametrize("offset, sign", [(0.8, -1), (-0.8, 1)])
def test_uturn_align_turns_toward_line_at_edge(offset, sign):
    """유턴 뒤 라인이 화면 끝(관성으로 지나침·덜 돎)에 있으면 그쪽으로 천천히 돎 — 오른쪽(+)이면 오른쪽(−)."""
    c = DriveConfig()
    d = aligning()
    out = d.step(at(offset), DT)
    assert not out.done and out.linear == 0 and out.angular * sign > 0
    assert c.align_min <= abs(out.angular) <= c.align_max and abs(out.angular) < c.turn_speed


def test_uturn_align_small_error_uses_min_speed():
    c = DriveConfig()
    d = aligning()
    out = d.step(at(0.15), DT)
    assert out.angular == pytest.approx(-c.align_min)


def test_uturn_align_ends_after_centered_frames():
    """화면 끝 → 가운데로 들어오면 멈추고, 멈춘 채 연속 맞으면 끝."""
    d = aligning()
    for off in (0.8, 0.5, 0.3):
        assert not d.step(at(off), DT).done
    out = d.step(at(0.05), DT)
    assert not out.done and out.angular == 0                  # 멈춰서 확인
    out = d.step(at(0.0), DT)
    assert out.done and out.result == "UTURN_DONE" and out.angular == 0


def test_uturn_align_uses_center_offset():
    """기준 위치(앱 "라인 기준")에 맞춤 — 기준이 −0.1이면 0.0은 아직 오른쪽."""
    d = aligning(center_offset=-0.1, align_tol=0.05)
    assert d.step(at(0.0), DT).angular < 0
    d.step(at(-0.1), DT)
    assert d.step(at(-0.1), DT).done


def test_uturn_align_overshoot_turns_back():
    """맞추다 지나쳐 반대쪽으로 가면 되돌아 돎."""
    d = aligning()
    assert d.step(at(0.5), DT).angular < 0
    assert d.step(at(-0.5), DT).angular > 0


def test_uturn_align_line_lost_turns_to_last_side():
    d = aligning()
    d.step(at(0.9), DT)
    out = d.step(GONE, DT)
    assert out.angular == pytest.approx(-DriveConfig().align_min)


def test_uturn_align_timeout_still_done():
    """못 맞춰도 시간이 지나면 끝 (라인은 찾았으므로 UTURN_DONE)."""
    d = aligning()
    out = None
    for _ in range(int(DriveConfig().align_timeout_s / DT) + 2):
        out = d.step(at(0.8), DT)
        if out.done:
            break
    assert out.done and out.result == "UTURN_DONE" and out.angular == 0


def test_uturn_t_bar_is_not_the_line():
    d = uturn()
    for _ in range(int(DriveConfig().uturn_min_s / DT) + 20):
        assert not d.step(BAR, DT).done


def test_uturn_timeout_is_e_turn():
    d = uturn()
    out = None
    for _ in range(int(DriveConfig().uturn_timeout_s / DT) + 1):
        out = d.step(GONE, DT)
        if out.done:
            break
    assert out.done and out.result == "E_TURN" and out.angular == 0 and not d.active


def test_uturn_ignores_line_events_and_markers():
    d = uturn()
    d.set_target("red")
    stop = LineInput(0.0, True, LineEvent.STOP_LINE, LineEvent.STOP_LINE)
    assert not d.step(stop, DT, see("left", y=200.0)).done and d.phase == "UTURN"


def test_uturn_cancel_stops():
    d = uturn()
    out = d.cancel()
    assert out.done and out.result == "STOPPED" and out.angular == 0 and d.phase == "FOLLOW"


def test_uturn_line_timeout():
    d = uturn()
    out = None
    for _ in range(int(DriveConfig().line_timeout_s / DT) + 2):
        out = d.step(None, DT)
        if out.done:
            break
    assert out.done and out.result == "E_LINE"


# ── 정지선 통과·표지판 접근 (2026-11-02) ──────────────────

STOP = LineInput(0.0, True, LineEvent.STOP_LINE, LineEvent.STOP_LINE, 90.0, True)
STOP_RAW = LineInput(0.0, True, LineEvent.NONE, LineEvent.STOP_LINE, 90.0, True)


def test_skip_stop_line_passes_one_then_stops_at_next():
    d = DriveController()
    d.start(DriveCommand.FOLLOW_TO_EVENT, skip_stop_lines=1)
    for line in (STOP_RAW, STOP, STOP):
        out = d.step(line, DT)
        assert not out.done and out.linear > 0 and d.phase == "PASS STOP_LINE"
    for _ in range(DriveConfig().pass_clear_frames):
        d.step(AHEAD, DT)
    assert d.skip_stop == 0 and d.phase == "FOLLOW"
    out = d.step(STOP_RAW, DT)                                 # 다음 정지선은 멈춤
    assert out.linear == 0 and d.phase == "VERIFY STOP_LINE"


def test_skip_stop_line_flicker_is_one_line():
    """통과 중 판정이 잠깐 빠졌다 다시 나와도 같은 정지선."""
    d = DriveController()
    d.start(DriveCommand.FOLLOW_TO_EVENT, skip_stop_lines=1)
    for line in (STOP, AHEAD, AHEAD, STOP, AHEAD, STOP):
        assert not d.step(line, DT).done
    assert d.skip_stop == 1


def test_skip_does_not_hide_other_events():
    d = DriveController()
    d.start(DriveCommand.FOLLOW_TO_EVENT, skip_stop_lines=1)
    out = d.step(LineInput(0.0, True, LineEvent.CORNER_LEFT, LineEvent.CORNER_LEFT), DT)
    assert out.done and out.result == "CORNER_LEFT"


def test_without_skip_stop_line_still_stops():
    d = DriveController()
    d.start(DriveCommand.FOLLOW_TO_EVENT)
    assert d.step(STOP, DT).done


def test_approach_slow_and_ignores_events():
    c = DriveConfig()
    d = DriveController()
    d.set_target("red")
    d.start(DriveCommand.APPROACH)
    for _ in range(int(c.approach_kick_s / DT) + 1):
        d.step(AHEAD, DT)
    for line in (STOP, LineInput(0.0, True, LineEvent.LINE_END, LineEvent.LINE_END), BAR):
        out = d.step(line, DT, see("left", y=200.0))
        assert not out.done and out.linear == pytest.approx(c.approach_speed) and d.phase == "APPROACH"


def test_approach_kicks_then_slows():
    """멈춘 TT 모터가 움직이도록 처음 0.3초는 시동 세기(0.45), 그 뒤 접근 속도(0.35)."""
    c = DriveConfig()
    assert c.approach_speed < c.approach_kick_speed
    d = DriveController()
    d.start(DriveCommand.APPROACH)
    assert d.step(AHEAD, DT).linear == pytest.approx(c.approach_kick_speed)
    for _ in range(int(c.approach_kick_s / DT)):
        d.step(AHEAD, DT)
    assert d.step(AHEAD, DT).linear == pytest.approx(c.approach_speed)


def test_approach_kick_again_on_restart():
    c = DriveConfig()
    d = DriveController()
    d.start(DriveCommand.APPROACH)
    for _ in range(20):
        d.step(AHEAD, DT)
    d.cancel()
    d.start(DriveCommand.APPROACH)
    assert d.step(AHEAD, DT).linear == pytest.approx(c.approach_kick_speed)


def test_approach_speed_tune_from_app():
    d = DriveController()
    d.set_tune("APPROACH", 0.3)
    d.start(DriveCommand.APPROACH)
    for _ in range(20):
        out = d.step(AHEAD, DT)
    assert out.linear == pytest.approx(0.3)


def test_approach_faster_than_kick_uses_approach_speed():
    d = DriveController()
    d.set_tune("APPROACH", 0.6)
    d.start(DriveCommand.APPROACH)
    assert d.step(AHEAD, DT).linear == pytest.approx(0.6)


def test_approach_goes_straight_ignoring_line():
    """카메라가 표지판 쪽으로 올라가 있어 보이는 '라인'은 믿을 수 없음 → 직진 (2026-11-06 순서 변경)."""
    d = DriveController()
    d.start(DriveCommand.APPROACH)
    out = d.step(LineInput(0.4, True, angle=60.0, angle_valid=True), DT)
    assert out.angular == 0 and out.linear == pytest.approx(DriveConfig().approach_kick_speed)
    out = d.step(LineInput(0.4, False), DT)
    assert out.angular == 0 and out.linear > 0


def test_approach_cancel():
    d = DriveController()
    d.start(DriveCommand.APPROACH)
    d.step(AHEAD, DT)
    out = d.cancel()
    assert out.done and out.result == "STOPPED" and out.linear == 0


# ── 라인 위 마커 (2026-11-06): center = 직진, 갈래 위 = 회전 ──

def test_center_marker_goes_straight_without_stopping():
    c = DriveConfig()
    d = marker_drive()
    assert d.step(AHEAD, DT, see("center", y=80.0)).linear == pytest.approx(c.speed)    # 감속도 안 함
    out = d.step(AHEAD, DT, see("center", y=130.0))
    assert out.linear > 0 and d.phase == "FOLLOW" and d.turn_dir == 0
    assert d.decision == "STRAIGHT (앞 라인 위 마커)"
    assert d.marker_ignore == pytest.approx(c.marker_ignore_s)


def test_branch_marker_stops_and_turns_even_with_other_color_center():
    """사진 배치: 노랑(앞 라인 위, 다른 색)과 빨강(왼쪽 갈래, 목적지) → 좌회전."""
    d = marker_drive("red")
    markers = see("left") + [MarkerInput("yellow", "center", 160.0)]
    out = d.step(AHEAD, DT, markers)
    assert out.linear == 0 and d.phase == "MARKER red"
    check(d, "left")
    assert d.phase == "TURN LEFT"


def test_center_frames_during_check_mean_straight():
    """멈춰서 보니 갈래가 아니라 앞 라인 위로 확인되면 직진."""
    d = marker_drive()
    d.step(AHEAD, DT, see("left"))
    for _ in range(40):
        d.step(branches(left=True), DT, see("center"))
        if not d.phase.startswith("MARKER"):
            break
    assert d.phase == "FOLLOW" and d.turn_dir == 0


def test_other_color_center_is_ignored():
    d = marker_drive("red")
    d.step(AHEAD, DT, [MarkerInput("blue", "center", 200.0)])
    assert d.decision == "" and d.marker_ignore == 0


# ── 초음파 정지 ─────────────────────────────────────────

def ranges(d, values):
    out = None
    for v in values:
        d.set_range(v)
        out = d.step(AHEAD, DT)
        if out.done:
            break
    return out


def test_obstacle_stops_after_three_close_readings():
    d = follow()
    out = ranges(d, [0.17, 0.18])
    assert not out.done and out.linear > 0
    out = ranges(d, [0.1])
    assert out.done and out.result == "OBSTACLE" and out.linear == 0 and out.angular == 0
    assert not d.active


def test_obstacle_needs_consecutive_readings():
    d = follow()
    out = ranges(d, [0.1, 0.1, 0.3, 0.1, 0.1, 0.5, 0.15, 0.15])
    assert not out.done and d.active


def test_obstacle_missing_reading_does_not_reset():
    """값 없음(측정 실패)은 건너뜀 — mission_node 접근과 같은 규칙."""
    d = follow()
    out = ranges(d, [0.1, None, 0.1, None, 0.1])
    assert out.done and out.result == "OBSTACLE"


def test_obstacle_far_never_stops():
    d = follow()
    out = ranges(d, [0.19] * 20)
    assert not out.done and out.linear > 0


def test_obstacle_off():
    d = DriveController(DriveConfig(obstacle_stop=False))
    d.start(DriveCommand.FOLLOW_TO_EVENT)
    assert not ranges(d, [0.05] * 10).done


def test_obstacle_ignored_during_uturn():
    """유턴은 표지판을 보고 선 자리에서 시작 — 가까워도 멈추지 않음."""
    d = uturn()
    for _ in range(10):
        d.set_range(0.1)
        assert not d.step(GONE, DT).done


def test_obstacle_ignored_during_approach():
    """APPROACH는 mission_node가 표지판 거리로 멈춤 — drive는 보지 않음."""
    d = DriveController(DriveConfig())
    d.start(DriveCommand.APPROACH)
    for _ in range(10):
        d.set_range(0.1)
        assert not d.step(AHEAD, DT).done


def test_obstacle_ignored_while_spinning_for_marker_turn():
    """마커 회전의 제자리 회전 중에는 보지 않고, 회전 전 전진 중에는 봄."""
    d = turning()
    assert d.turn_stage in ("LEAVE", "BLIND")
    for _ in range(5):
        d.set_range(0.1)
        assert not d.step(GONE, DT).done
    f = marker_drive()
    f.step(AHEAD, DT, see("left"))
    check(f, "left")
    assert f.turn_stage == "FORWARD"
    out = ranges(f, [0.1, 0.1, 0.1])
    assert out.done and out.result == "OBSTACLE"


def test_obstacle_count_resets_on_new_start():
    d = follow()
    ranges(d, [0.1, 0.1])
    d.cancel()
    d.start(DriveCommand.FOLLOW_TO_EVENT)
    assert not ranges(d, [0.1]).done


def test_notes_verify_start_and_confirm():
    """정지선 처리 기록 (drive_node 로그) — 멈추고 확인, 확정."""
    d = follow()
    d.step(RAW_STOP, DT)
    assert "멈추고 확인" in d.note and d.note_no == 1
    d.step(CONFIRMED_STOP, DT)
    assert "확정" in d.note and d.note_no == 2


def test_notes_verify_failure():
    d = follow()
    d.step(RAW_STOP, DT)
    for _ in range(5):
        d.step(CLEAR, DT)
    assert "확인 실패" in d.note and "사라짐" in d.note


def test_notes_stop_line_ignored_after_turn_once():
    d = finished_turn()
    stop = LineInput(0.0, True, LineEvent.STOP_LINE, LineEvent.STOP_LINE)
    d.step(stop, DT)
    d.step(stop, DT)
    assert "회전 직후" in d.note and d.note_no == 1


def test_skip_expires_when_stop_line_never_seen():
    """2026-11-09: 출발 때 통과할 정지선이 이미 카메라 밖이면 pass_window_s 뒤 통과 횟수 0 → 목적지 정지선에서 멈춤."""
    d = DriveController()
    d.start(DriveCommand.FOLLOW_TO_EVENT, skip_stop_lines=1)
    for _ in range(int(DriveConfig().pass_window_s / DT) + 1):
        d.step(AHEAD, DT)
    assert d.skip_stop == 0 and "이미 지난" in d.note
    out = d.step(STOP_RAW, DT)
    assert out.linear == 0 and d.phase == "VERIFY STOP_LINE"


def test_skip_does_not_expire_once_stop_line_seen():
    """통과할 정지선을 보기 시작했으면 시간이 지나도 그 정지선을 끝까지 통과로 처리."""
    d = DriveController()
    d.start(DriveCommand.FOLLOW_TO_EVENT, skip_stop_lines=1)
    for _ in range(int(DriveConfig().pass_window_s / DT) - 2):
        d.step(AHEAD, DT)
    for _ in range(10):
        assert not d.step(STOP, DT).done
    assert d.skip_stop == 1 and d.phase == "PASS STOP_LINE"


def test_skip_window_off():
    d = DriveController(DriveConfig(pass_window_s=0))
    d.start(DriveCommand.FOLLOW_TO_EVENT, skip_stop_lines=1)
    for _ in range(200):
        d.step(AHEAD, DT)
    assert d.skip_stop == 1


# ── 동작 로그 (앱 콘솔, 2026-11-09) ──
def logs_of(d, kind):
    return [t for k, t in d.take_logs() if k == kind]


def test_marker_logs_recognition_and_decision_with_votes():
    d = marker_drive()
    d.step(AHEAD, DT, see("left", y=80.0))                  # 감속 구간
    d.step(AHEAD, DT, see("left"))                          # 정지 구간
    check(d, "left")
    texts = logs_of(d, "MARKER")
    assert any("마커 인식: red 왼쪽" in t and "감속" in t for t in texts)
    assert any("멈추고 좌우 확인" in t for t in texts)
    assert any(t.startswith("마커 판단: LEFT [스티커 왼쪽") for t in texts)
    assert d.take_logs() == []                              # 비우면 다시 안 나옴


def test_straight_marker_decision_logged():
    d = marker_drive()
    d.step(AHEAD, DT, see("center"))
    assert logs_of(d, "MARKER") == ["마커 판단: STRAIGHT (앞 라인 위 마커)"]


def test_stop_line_note_goes_to_stop_log():
    d = DriveController()
    d.start(DriveCommand.FOLLOW_TO_EVENT)
    d.step(LineInput(0.0, True, LineEvent.NONE, LineEvent.STOP_LINE), DT)
    assert any(t.startswith("정지선: STOP_LINE 보임") for t in logs_of(d, "STOP"))


def test_line_timeout_logs_reason():
    d = DriveController()
    d.start(DriveCommand.FOLLOW_TO_EVENT)
    out = None
    for _ in range(20):
        out = d.step(None, DT)
        if out.done:
            break
    assert out.result == "E_LINE" and any("라인 영상" in t for t in logs_of(d, "STOP"))


def test_logs_are_capped():
    d = DriveController()
    for i in range(80):
        d._log("STOP", str(i))
    assert len(d.logs) == 50 and d.logs[-1] == ("STOP", "79")

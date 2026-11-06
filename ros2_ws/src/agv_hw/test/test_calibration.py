"""PanCalibrator — 가짜 카메라(팬 → 라인 각도)로 정면 자동 맞춤 시험."""
import math

import pytest

from agv_hw.calibration import CalConfig, PanCalibrator

DT = 0.1


def camera(front, gain=-1.5, curve=False):
    """팬 각도 → 화면 라인 각도. 정면이면 90°. gain = 팬 1°당 각도 변화 (부호는 서보 방향에 따라 다름)."""
    def angle(pan):
        d = pan - front
        if curve:                                        # 실제처럼 멀어질수록 덜 변함
            return 90 + 40 * math.tanh(gain * d / 40)
        return 90 + gain * d
    return angle


def run(cal, start_pan, angle_of, valid=lambda pan: True, limit=2000, noise=None):
    """서보·카메라를 흉내 냄 — MOVE를 받으면 팬을 옮기고, 라인 각도를 10Hz로 넣는다."""
    now = 0.0
    steps = [cal.start(start_pan, now)]
    pan = steps[0].pan
    for i in range(limit):
        now += DT
        a = angle_of(pan) + (noise(i) if noise else 0.0)
        for s in cal.feed(a, valid(pan), now) + [x for x in [cal.tick(now)] if x]:
            steps.append(s)
            if s.action == "MOVE":
                pan = s.pan
            if s.action in ("DONE", "FAIL"):
                return s, steps
    return None, steps


@pytest.mark.parametrize("front, start, gain", [(66, 70, -1.5), (66, 70, 1.5), (58, 70, -1.0), (75, 65, 2.0)])
def test_finds_front(front, start, gain):
    cal = PanCalibrator()
    end, steps = run(cal, start, camera(front, gain))
    assert end.action == "DONE" and abs(end.pan - front) <= 1.0
    assert abs(end.angle - 90) <= CalConfig().tolerance_deg


def test_finds_front_with_nonlinear_camera():
    end, _ = run(PanCalibrator(), 70, camera(55, -2.0, curve=True))
    assert end.action == "DONE" and abs(end.pan - 55) <= 1.0


def test_already_front_finishes_without_moving():
    end, steps = run(PanCalibrator(), 66, camera(66))
    assert end.action == "DONE" and end.pan == 66
    assert [s.action for s in steps if s.action == "MOVE"] == ["MOVE"]       # 시작 위치뿐


def test_median_ignores_outlier_frames():
    noise = lambda i: 40.0 if i % 7 == 0 else 0.0      # noqa: E731 — 가끔 크게 튀는 각도
    end, _ = run(PanCalibrator(), 70, camera(64), noise=noise)
    assert end.action == "DONE" and abs(end.pan - 64) <= 1.0


def test_waits_for_settle_before_measuring():
    cal = PanCalibrator(CalConfig(settle_s=0.5))
    cal.start(70, 0.0)
    assert cal.feed(90.0, True, 0.2) == [] and cal.samples == []


def test_line_not_visible_fails():
    end, _ = run(PanCalibrator(), 70, camera(66), valid=lambda pan: False)
    assert end.action == "FAIL" and "라인이 안 보임" in end.reason


def test_no_response_to_pan_fails():
    end, _ = run(PanCalibrator(), 70, lambda pan: 80.0)
    assert end.action == "FAIL" and "안 변함" in end.reason


def test_moves_are_limited_and_clamped():
    cfg = CalConfig(max_move_deg=10.0, pan_min=20.0, pan_max=120.0)
    end, steps = run(PanCalibrator(cfg), 30, camera(5, -0.5))      # 정면이 범위 밖
    moves = [s.pan for s in steps if s.action == "MOVE"]
    assert all(b - a <= 10.0 + 1e-9 and a - b <= 10.0 + 1e-9 for a, b in zip(moves, moves[1:]))
    assert min(moves) >= 20.0 and end.action == "FAIL"


def test_timeout():
    cal = PanCalibrator(CalConfig(timeout_s=5.0))
    cal.start(70, 0.0)
    assert cal.tick(4.9) is None
    assert cal.tick(5.1).action == "FAIL" and not cal.active


def test_reports_each_measurement():
    end, steps = run(PanCalibrator(), 70, camera(66))
    measured = [s for s in steps if s.action == "MEASURED"]
    assert measured[0].pan == 70 and measured[0].angle == pytest.approx(84.0)
    assert len(measured) >= 2

"""LineDetector — 합성 트랙 사진(320×240)으로 오프셋·이벤트 시험. 관심 영역 = 아래 80행(160~239)."""
import numpy as np
import pytest

from agv_protocol.types import LineEvent
from agv_vision.line_detect import EventConfirmer, LineConfig, LineDetector

W, H, LW = 320, 240, 20           # 라인 굵기 20px
TOP = 160                          # 관심 영역 시작 행 → 먼 띠 160~185, 중간 186~212, 가까운 띠 213~239
MID_Y = (192, 206)                 # 중간 띠 안의 가로 라인


def floor():
    img = np.full((H, W, 3), 225, np.uint8)
    rng = np.random.default_rng(0)
    return np.clip(img + rng.integers(-8, 8, img.shape), 0, 255).astype(np.uint8)


def vline(img, x, y0=0, y1=H):
    img[y0:y1, x - LW // 2: x + LW // 2] = 25


def hline(img, x0, x1, y=MID_Y):
    img[y[0]:y[1], x0:x1] = 25


def run(img, frames=3):
    det = LineDetector(LineConfig())
    for _ in range(frames):
        r = det.process(img)
    return r


def test_straight_center():
    img = floor()
    vline(img, 160)
    r = run(img)
    assert r.line_found and abs(r.offset) < 0.05 and r.event == LineEvent.NONE


@pytest.mark.parametrize("x, sign", [(230, 1), (90, -1)])
def test_offset_sign(x, sign):
    img = floor()
    vline(img, x)
    r = run(img)
    assert r.line_found and np.sign(r.offset) == sign
    assert abs(r.offset - (x - 160) / 160) < 0.05


def test_corner_left():
    img = floor()
    vline(img, 160, MID_Y[0], H)          # 라인이 가로선에서 끝남 (앞쪽 없음)
    hline(img, 0, 170)
    assert run(img).event == LineEvent.CORNER_LEFT


def test_corner_right():
    img = floor()
    vline(img, 160, MID_Y[0], H)
    hline(img, 150, W)
    assert run(img).event == LineEvent.CORNER_RIGHT


def junction_t():
    img = floor()
    vline(img, 160, MID_Y[0], H)
    hline(img, 0, W)
    return img


@pytest.mark.parametrize("make", [
    junction_t,
    lambda: (lambda img: (vline(img, 160), hline(img, 0, 170), img)[-1])(floor()),     # 왼쪽 옆 갈래
    lambda: (lambda img: (vline(img, 160), hline(img, 150, W), img)[-1])(floor()),     # 오른쪽 옆 갈래
], ids=["T", "side_left", "side_right"])
def test_junctions_are_left_to_markers(make):
    """갈림길은 색 마커 인식이 맡음 (2026-10-30) — 라인 검출은 이벤트를 내지 않고 라인만 따라감."""
    r = run(make())
    assert r.line_found and r.event == LineEvent.NONE and abs(r.offset) < 0.1


def test_stop_line():
    img = floor()
    vline(img, 160)
    hline(img, 110, 210)                  # 폭 100px = 라인 굵기 5배 이내
    assert run(img).event == LineEvent.STOP_LINE


def test_line_end():
    r = run(floor())
    assert not r.line_found and r.event == LineEvent.LINE_END


def test_colored_markers_are_not_line():
    """빨강·파랑 마커(어둡게 보이는 색)가 라인 옆에 있어도 오프셋은 라인 기준."""
    img = floor()
    vline(img, 160)
    img[195:240, 40:100] = (15, 10, 90)         # 어두운 빨강 (BGR, V 90 — 밝기만으로는 검정과 구분 안 됨)
    img[195:240, 220:280] = (90, 30, 5)         # 어두운 파랑
    r = run(img)
    assert abs(r.offset) < 0.05 and r.event == LineEvent.NONE


def touching_markers():
    """라인에 바로 붙은 어두운 마커 (갈래 시작점처럼) — 떨어진 물체와 달리 라인과 한 덩어리가 될 수 있음."""
    img = floor()
    vline(img, 160)
    img[195:240, 170:240] = (15, 10, 90)          # 오른쪽에 붙은 어두운 빨강
    return img


def test_touching_marker_is_not_line():
    r = run(touching_markers())
    assert abs(r.offset) < 0.05 and r.event == LineEvent.NONE


def test_without_saturation_filter_touching_marker_breaks_detection():
    """채도 조건이 실제로 필요함을 확인 (끄면 붙은 마커가 라인과 합쳐져 오프셋이 밀림)."""
    det = LineDetector(LineConfig(black_s_max=255))
    for _ in range(3):
        r = det.process(touching_markers())
    assert abs(r.offset) > 0.1 or r.raw_event != LineEvent.NONE


@pytest.mark.parametrize("x0, x1", [(0, 70), (220, 320), (5, 130)])
def test_large_dark_object_beside_line(x0, x1):
    """옆의 큰 어두운 물체(아래까지 닿음)가 있어도 가운데 라인을 고르고, 이벤트로 잡지 않는다."""
    img = floor()
    vline(img, 160)
    img[150:240, x0:x1] = 30
    if x1 > 140:                                  # 라인과 붙지 않게 틈
        img[150:240, 140 - 15:140] = 225
    r = run(img)
    assert r.line_found and abs(r.offset) < 0.05 and r.event == LineEvent.NONE


def test_dark_object_does_not_hide_corner():
    """옆 물체가 있어도 라인과 이어진 코너는 그대로 판정."""
    img = floor()
    vline(img, 160, MID_Y[0], H)
    hline(img, 150, W)                            # 오른쪽으로 꺾임
    img[215:240, 0:60] = 30                       # 왼쪽 아래 어두운 물체 (떨어져 있음)
    assert run(img).event == LineEvent.CORNER_RIGHT


def test_side_object_only_is_line_end():
    img = floor()
    img[170:240, 0:60] = 30
    r = run(img)
    assert r.event == LineEvent.LINE_END or not r.line_found or abs(r.offset) > 0.5


def test_keeps_last_offset_when_line_lost():
    det = LineDetector(LineConfig())
    img = floor()
    vline(img, 230)
    det.process(img)
    r = det.process(floor())
    assert not r.line_found and r.offset > 0.3


def test_event_needs_consecutive_frames():
    img = floor()
    vline(img, 160, MID_Y[0], H)
    hline(img, 0, 170)
    det = LineDetector(LineConfig())
    events = [det.process(img).event for _ in range(3)]
    assert events == [LineEvent.NONE, LineEvent.NONE, LineEvent.CORNER_LEFT]


def test_confirmer_resets_on_change():
    c = EventConfirmer(2)
    assert c.push(LineEvent.STOP_LINE) == LineEvent.NONE
    assert c.push(LineEvent.CORNER_LEFT) == LineEvent.NONE
    assert c.push(LineEvent.CORNER_LEFT) == LineEvent.CORNER_LEFT

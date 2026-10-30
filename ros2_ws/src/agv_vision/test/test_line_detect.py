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


def test_junction_t():
    img = floor()
    vline(img, 160, MID_Y[0], H)
    hline(img, 0, W)
    assert run(img).event == LineEvent.JUNCTION_T


def test_junction_left_with_line_ahead():
    img = floor()
    vline(img, 160)                       # 앞쪽 라인 계속
    hline(img, 0, 170)
    assert run(img).event == LineEvent.JUNCTION_LEFT


def test_junction_right_with_line_ahead():
    img = floor()
    vline(img, 160)
    hline(img, 150, W)
    assert run(img).event == LineEvent.JUNCTION_RIGHT


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


def test_without_saturation_filter_markers_break_detection():
    """채도 조건이 실제로 필요함을 확인 (끄면 마커가 라인·가로선으로 잡힘)."""
    img = floor()
    vline(img, 160)
    img[195:240, 40:100] = (15, 10, 90)
    img[195:240, 220:280] = (90, 30, 5)
    det = LineDetector(LineConfig(black_s_max=255))
    for _ in range(3):
        r = det.process(img)
    assert r.raw_event != LineEvent.NONE or abs(r.offset) > 0.05


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
    hline(img, 0, W)
    det = LineDetector(LineConfig())
    events = [det.process(img).event for _ in range(3)]
    assert events == [LineEvent.NONE, LineEvent.NONE, LineEvent.JUNCTION_T]


def test_confirmer_resets_on_change():
    c = EventConfirmer(2)
    assert c.push(LineEvent.STOP_LINE) == LineEvent.NONE
    assert c.push(LineEvent.CORNER_LEFT) == LineEvent.NONE
    assert c.push(LineEvent.CORNER_LEFT) == LineEvent.CORNER_LEFT

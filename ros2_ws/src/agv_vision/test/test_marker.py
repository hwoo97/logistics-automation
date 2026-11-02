"""MarkerDetector — 합성 사진(320×240)으로 색·좌우·크기 시험. 색은 2026-11-02 실제 스티커 사진의 HSV 중앙값."""
import cv2
import numpy as np
import pytest

from agv_vision.marker import MarkerConfig, MarkerDetector
from agv_vision.stop_line import StopLineDetector

W, H = 320, 240
HSV = {"red": (176, 190, 155), "yellow": (30, 230, 190), "green": (72, 155, 125), "blue": (108, 235, 165)}


def bgr(color):
    return tuple(int(v) for v in cv2.cvtColor(np.uint8([[HSV[color]]]), cv2.COLOR_HSV2BGR)[0, 0])


def floor(line_x=160):
    img = np.full((H, W, 3), 215, np.uint8)
    img[:, line_x - 12: line_x + 12] = 25
    return img


def sticker(img, color, x, y, d=None):
    """y 높이에 맞는 크기(지름 ≈ 8.5 + 0.076y, 세로 0.8배)의 타원."""
    d = d or 8.5 + 0.076 * y
    cv2.ellipse(img, (x, y), (int(d / 2), int(d * 0.4)), 0, 0, 360, bgr(color), -1)
    return img


def detect(img, offset=0.0, angle=None):
    return MarkerDetector().process(img, offset, angle)


@pytest.mark.parametrize("color", HSV)
@pytest.mark.parametrize("y", [50, 120, 170, 220])
def test_each_color_any_height(color, y):
    found = detect(sticker(floor(), color, 60, y))
    assert [(m.color, m.side) for m in found] == [(color, "left")] and abs(found[0].y - y) <= 2


def test_side_is_relative_to_line_not_screen_center():
    """라인이 화면 오른쪽(x220)에 있으면 화면 가운데 오른쪽(x190)의 스티커도 라인 왼쪽."""
    img = sticker(floor(220), "red", 190, 170)
    assert detect(img, offset=220 / 160 - 1)[0].side == "left"
    img = sticker(floor(100), "red", 130, 170)
    assert detect(img, offset=100 / 160 - 1)[0].side == "right"


def test_tilted_line_uses_line_position_at_sticker_height():
    """라인 아래쪽 x160, 위쪽으로 오른쪽 기움(60°) → y100에서 라인은 x233. x200 스티커는 라인 왼쪽."""
    img = np.full((H, W, 3), 215, np.uint8)
    img = sticker(img, "green", 200, 100)
    assert detect(img, offset=0.0, angle=60.0)[0].side == "left"
    assert detect(img, offset=0.0, angle=None)[0].side == "right"


def test_both_sides_and_two_colors():
    img = floor()
    sticker(img, "blue", 100, 170)
    sticker(img, "blue", 220, 170)
    sticker(img, "red", 220, 130)
    found = {(m.color, m.side) for m in detect(img)}
    assert found == {("blue", "left"), ("blue", "right"), ("red", "right")}


@pytest.mark.parametrize("d", [4, 60])
def test_wrong_size_is_not_a_sticker(d):
    assert detect(sticker(floor(), "red", 70, 170, d=d)) == []


def test_long_tape_is_not_a_sticker():
    img = floor()
    img[160:176, 20:120] = bgr("blue")
    assert detect(img) == []


def test_black_line_and_gray_floor_are_not_markers():
    assert detect(floor()) == []


def test_pale_color_is_not_a_marker():
    """스티커 시트 배경 (옅은 청록, S 약 70)."""
    img = floor()
    pale = cv2.cvtColor(np.uint8([[(90, 70, 190)]]), cv2.COLOR_HSV2BGR)[0, 0]
    cv2.circle(img, (70, 170), 12, tuple(int(v) for v in pale), -1)
    assert detect(img) == []


def test_red_wraps_hue_zero():
    for hue in (2, 174):
        img = floor()
        color = cv2.cvtColor(np.uint8([[(hue, 190, 155)]]), cv2.COLOR_HSV2BGR)[0, 0]
        cv2.ellipse(img, (70, 170), (11, 9), 0, 0, 360, tuple(int(v) for v in color), -1)
        assert [m.color for m in detect(img)] == ["red"]


def test_input_is_resized():
    img = cv2.resize(sticker(floor(), "yellow", 60, 170), (640, 480), interpolation=cv2.INTER_NEAREST)
    assert [m.color for m in detect(img)] == ["yellow"]


def test_custom_range():
    cfg = MarkerConfig(red=(170, 5, 250, 250))       # 범위를 좁히면 안 잡힘
    assert MarkerDetector(cfg).process(sticker(floor(), "red", 60, 170), 0.0) == []


def test_stickers_do_not_become_stop_line():
    """라인 옆 스티커 양쪽이 있어도 정지선이 아님 (검정 판정은 색을 뺌)."""
    img = floor()
    for color in HSV:
        sticker(img, color, 120, 190)
        sticker(img, color, 200, 190)
    det = StopLineDetector()
    assert not any(det.process(img, 0.0).detected for _ in range(5))

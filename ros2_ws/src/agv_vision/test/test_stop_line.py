"""StopLineDetector — 합성 사진(320×240)으로 세로 열 정지선 판정 시험. 관심 영역 = 아래 120행(120~239)."""
import cv2
import numpy as np
import pytest

from agv_vision.stop_line import StopLineConfig, StopLineDetector

W, H, LW = 320, 240, 20           # 라인 굵기 20px
STOP_Y = (188, 212)               # 라인보다 두꺼운 정지선 (24px)


def floor():
    img = np.full((H, W, 3), 225, np.uint8)
    rng = np.random.default_rng(0)
    return np.clip(img + rng.integers(-8, 8, img.shape), 0, 255).astype(np.uint8)


def vline(img, x=160):
    img[:, x - LW // 2: x + LW // 2] = 25


def hline(img, x0, x1, y=STOP_Y):
    img[y[0]:y[1], x0:x1] = 25


def scene(x0=100, x1=220, y=STOP_Y, x=160):
    img = floor()
    vline(img, x)
    hline(img, x0, x1, y)
    return img


def run(img, offset=0.0, frames=3, **cfg):
    det = StopLineDetector(StopLineConfig(**cfg))
    for _ in range(frames):
        r = det.process(img, offset)
    return r


def rotated(angle, y=STOP_Y):
    """정지선 중심을 기준으로 회전 — 차량이 비스듬히 다가가거나 테이프가 삐뚤게 붙은 경우."""
    img = scene(y=y)
    m = cv2.getRotationMatrix2D((160, (y[0] + y[1]) / 2), angle, 1.0)
    return cv2.warpAffine(img, m, (W, H), borderMode=cv2.BORDER_REPLICATE)


def test_stop_line():
    r = run(scene())
    assert r.detected and r.confirmed and abs(r.angle) < 1


def test_no_stop_line_on_plain_line():
    img = floor()
    vline(img)
    r = run(img)
    assert not r.detected and not r.confirmed


def test_needs_confirm_frames():
    det = StopLineDetector(StopLineConfig(confirm_frames=3))
    results = [det.process(scene(), 0.0) for _ in range(3)]
    assert [r.detected for r in results] == [True] * 3
    assert [r.confirmed for r in results] == [False, False, True]


def test_set_frames_applies_immediately():
    det = StopLineDetector()
    det.set_frames(1)
    assert det.process(scene(), 0.0).confirmed


@pytest.mark.parametrize("angle", [-20, -15, -10, -5, 5, 10, 15, 20])
def test_tilted_stop_line(angle):
    """기울어도 잡음 — 가로 줄 방식은 +5°부터 대부분, ±15°부터 전부 놓쳤음 (2026-11-02 실제 사진)."""
    r = run(rotated(angle))
    assert r.detected, f"{angle}° 놓침"
    assert abs(abs(r.angle) - abs(angle)) < 5


def test_too_tilted_is_not_stop_line():
    assert not run(rotated(50)).detected


@pytest.mark.parametrize("x0, x1", [(85, 235), (40, 270)])
def test_wide_stop_line(x0, x1):
    assert run(scene(x0, x1)).detected


@pytest.mark.parametrize("x0, x1", [(135, 185)])
def test_too_short_is_not_stop_line(x0, x1):
    """라인 폭 2.5배 — 양옆으로 충분히 뻗지 않음."""
    assert not run(scene(x0, x1)).detected


@pytest.mark.parametrize("y", [165, 180, 195, 210, 220])
def test_found_anywhere_in_roi(y):
    assert run(scene(y=(y, y + 15))).detected


def test_entering_roi_is_not_judged_yet():
    """관심 영역 위 경계에 걸친 정지선은 아직 판정 안 함 (경계에서 멈추면 떨림으로 판정이 사라짐)."""
    assert not run(scene(y=(110, 140))).detected
    assert run(scene(y=(122, 152))).detected


def test_same_tape_foreshortened():
    """라인과 같은 테이프가 원근으로 0.75배로 보임 (실측 0.76~0.79배)."""
    assert run(scene(y=(192, 207))).detected


@pytest.mark.parametrize("thick", [6, 9, 12])
def test_thin_floor_line_is_not_stop_line(thick):
    """바닥의 얇은 가로선(줄눈)이 라인을 가로질러도 정지선 아님 — 기준 라인 20px × 0.65 = 13px."""
    assert not run(scene(y=(196, 196 + thick))).detected


def test_thin_diagonal_floor_line_is_not_stop_line():
    """기운 줄눈도 두께가 모자라 정지선 아님 — 열 방식이 각도에 강해진 만큼 확인."""
    img = floor()
    vline(img)
    cv2.line(img, (60, 215), (260, 185), (40, 40, 40), 5)
    assert not run(img).detected


def test_one_sided_mark_is_not_stop_line():
    img = floor()
    vline(img)
    hline(img, 160, 250)
    assert not run(img).detected


@pytest.mark.parametrize("x0, x1", [(0, 320)])
def test_t_junction_is_not_stop_line(x0, x1):
    """화면 양끝까지 = T자·갈림길 → 마커 인식이 맡음."""
    assert not run(scene(x0, x1)).detected


def test_off_center_stop_line_touching_one_edge():
    """차량이 비켜 있어 정지선 한쪽만 화면 끝에 닿아도 정지선 (가로 줄 방식은 놓쳤음)."""
    r = run(scene(0, 180, x=90), offset=(90 - 160) / 160)
    assert r.detected


def test_separate_object_beside_line_is_not_stop_line():
    """라인과 떨어진 어두운 물체는 정지선 아님."""
    img = floor()
    vline(img)
    img[185:215, 40:135] = 30
    img[185:215, 185:280] = 30
    assert not run(img).detected


@pytest.mark.parametrize("x_bottom, x_top", [(60, 300), (260, 20)])
def test_diagonal_curve_is_not_stop_line(x_bottom, x_top):
    """급한 곡선의 대각선 라인 — 세로 열로는 두께 있는 띠처럼 보이지만 정지선 아님."""
    img = floor()
    cv2.line(img, (x_bottom, H - 1), (x_top, 110), (25, 25, 25), LW)
    near_x = x_bottom + (x_top - x_bottom) * (H - 1 - 226) / (H - 1 - 110)
    assert not run(img, offset=(near_x - 160) / 160).detected


@pytest.mark.parametrize("offset", [0.0, 0.17, 0.25, -0.2])
def test_line_offset_pulled_by_stop_line(offset):
    """정지선이 가까운 띠에 들어오면 라인 오프셋이 끌려감 — 실제 사진: 라인 x172인데 오프셋은 x200 (2026-11-02)."""
    assert run(scene(120, 220, y=(200, 228), x=172), offset=offset).detected


def test_held_while_leaving_bottom():
    """지연·관성으로 지나쳐 멈춰 정지선이 영역 아래에 닿아도 판정 유지 — 사라지면 오판정으로 보고 재출발함 (2026-11-02 S1)."""
    det = StopLineDetector()
    assert det.process(scene(), 0.0).detected
    assert det.process(scene(y=(222, 240)), 0.0).detected
    assert not run(scene(y=(222, 240))).detected              # 처음부터 아래에 닿으면 판정 안 함


def test_hold_expires():
    det = StopLineDetector(StopLineConfig(hold_frames=2))
    det.process(scene(), 0.0)
    plain = floor()
    vline(plain)
    det.process(plain, 0.0)
    det.process(plain, 0.0)
    assert not det.process(scene(y=(222, 240)), 0.0).detected


def test_held_when_line_ends_at_stop_line():
    """라인이 정지선에서 끝나 위쪽에 라인 열이 없어도, 판정 직후라면 유지."""
    det = StopLineDetector()
    det.process(scene(), 0.0)
    img = floor()
    hline(img, 100, 220, (222, 240))
    assert det.process(img, 0.0).detected


def test_gap_in_tape_is_tolerated():
    """테이프 반사로 정지선이 몇 열 끊겨도 이어진 것으로 봄 (실제 사진 y132 부근)."""
    img = scene()
    img[STOP_Y[0]:STOP_Y[1], 200:202] = 225
    assert run(img).detected


def test_no_line_nearby_returns_false():
    assert not run(floor()).detected

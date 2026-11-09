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


def test_reason_empty_when_detected():
    r = run(scene())
    assert r.detected and r.reason == "" and not r.candidate


def test_reason_for_one_sided_mark():
    """진단 로그 — 놓칠 뻔한 장면은 후보로 표시하고 탈락 이유를 남김."""
    img = floor()
    vline(img)
    hline(img, 160, 250)
    r = run(img)
    assert not r.detected and r.candidate and "왼쪽" in r.reason


def test_reason_for_too_wide():
    r = run(scene(30, 290))
    assert not r.detected and r.candidate and r.reason.startswith("폭")


def test_plain_line_is_not_candidate():
    img = floor()
    vline(img)
    r = run(img)
    assert not r.detected and not r.candidate


# ── 놓침 의심 구간 (진단 로그, 2026-11-09) ──
from agv_vision.stop_line import MissTracker, StopLineResult  # noqa: E402

CAND = StopLineResult(False, False, 0.0, "한쪽이 짧음 (왼 6px)", True)
DET = StopLineResult(True, False)
NONE = StopLineResult(False, False)


def test_miss_reported_once_when_candidates_end_without_detection():
    m = MissTracker(3)
    assert [m.push(CAND) for _ in range(4)] == [None] * 4
    assert m.push(NONE) is None                                 # 한 장 빠짐은 같은 구간
    frames, top, last = m.push(NONE)
    assert frames == 4 and top == [("한쪽이 짧음", 4)] and last.startswith("한쪽이 짧음")
    assert m.push(NONE) is None


def test_no_miss_when_detected_in_episode():
    m = MissTracker(3)
    for r in (CAND, CAND, DET, CAND, NONE, NONE):
        assert m.push(r) is None


def test_short_noise_is_not_miss():
    m = MissTracker(3)
    for r in (CAND, CAND, NONE, NONE):
        assert m.push(r) is None


# ── 2026-11-09 교수 점검 반영 (①~④) ──
def perspective_scene(stop=(131, 145)):
    """원근 — 라인 굵기 ≈ 0.062 × (행 + 188)px (교수 점검 실측: 아래 26px, 위 12px). 먼 정지선은 얇게 보임."""
    img = floor()
    for y in range(H):
        half = 0.062 * (y + 188) / 2
        img[y, int(160 - half): int(160 + half)] = 25
    img[stop[0]:stop[1], 100:220] = 25
    return img


def test_far_stop_line_judged_with_local_line_width():
    """① 정지선 높이의 라인 굵기(≈20.7px)로 판정 — 관심 영역 전체 중앙값(≈23px)이면 두께 14줄이 기준 미달."""
    assert run(perspective_scene()).detected
    det = StopLineDetector()
    det.process(perspective_scene(), 0.0)
    assert det.last_local is not None and 19 <= det.last_local <= 22


def test_held_with_lower_thickness():
    """② 판정 직후는 두께 기준 0.5 — 처음부터 그 두께면 판정 안 함 (0.65 미달)."""
    thin = scene(y=(195, 207))                    # 12줄 = 라인 20px × 0.6
    det = StopLineDetector()
    assert det.process(scene(), 0.0).detected
    assert det.process(thin, 0.0).detected
    assert not run(thin).detected


def sticker(img, x0, x1, y0, y1, bgr=(30, 30, 220)):
    img[y0:y1, x0:x1] = bgr


def test_sticker_on_cross_line_is_junction():
    """④ 가로선 위 스티커 = 분기점 (색 확인). 분기점 확인을 끄면 정지선으로 판정 (③으로 스티커도 가로선에 포함)."""
    img = scene()
    sticker(img, 104, 128, *STOP_Y)
    r = run(img)
    assert not r.detected and r.reason.startswith("분기점") and r.candidate
    assert run(img, junction_check=False).detected


def test_cross_line_continuing_beyond_ends_is_junction():
    """④ 가로선이 양끝 너머로 이어지면(가늘어 정지선 열로는 안 잡혀도) 분기점 (이어짐 확인)."""
    img = scene()
    img[194:206, 30:100] = 25                     # 12줄 — 두께 기준(13) 미달이라 정지선 폭에는 안 들어감 (화면 끝까지 덮으면 그 줄 바닥이 검정이 됨)
    img[194:206, 220:290] = 25
    r = run(img)
    assert not r.detected and r.reason.startswith("분기점")
    assert run(img, frames=1, junction_check=False).detected   # 첫 프레임 — 다음부터는 ② 낮은 기준으로 가는 선까지 폭에 들어감


def test_sticker_ahead_of_stop_line_is_not_junction():
    """정지선 2w 앞 라인 위 스티커는 분기점 아님 — 색 확인은 정지선 세로 범위(± 0.25 라인 굵기)만 봄."""
    img = scene()
    sticker(img, 150, 170, 140, 160)
    assert run(img).detected


@pytest.mark.parametrize("y0", [121, 123, 125])
def test_stop_line_just_entering_roi_top_is_not_junction(y0):
    """관심 영역 맨 위로 막 들어온 정지선(처음 판정되는 위치)을 분기점으로 보지 않음 — 중심선 위아래 띠가 위로 잘려도 이어짐 아님."""
    r = run(scene(y=(y0, y0 + 14)), frames=1)
    assert r.detected, r.reason


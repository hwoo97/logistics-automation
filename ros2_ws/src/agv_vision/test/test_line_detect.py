"""LineDetector — 합성 트랙 사진(320×240)으로 오프셋·이벤트 시험. 관심 영역 = 아래 80행(160~239)."""
import numpy as np
import pytest

from agv_protocol.types import LineEvent
from agv_vision.line_detect import EventConfirmer, LineConfig, LineDetector

W, H, LW = 320, 240, 20           # 라인 굵기 20px
TOP = 160                          # 관심 영역 시작 행 → 먼 띠 160~185, 중간 186~212, 가까운 띠 213~239
MID_Y = (192, 206)                 # 중간 띠 안의 가로 라인 (코너·갈래, 라인 테이프 굵기)
STOP_Y = (188, 212)                # 정지선 — 40~50mm 테이프라 라인보다 두꺼움 (24px)


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
    hline(img, 110, 210, STOP_Y)          # 폭 100px = 라인 굵기 5배, 두께 24px
    assert run(img).event == LineEvent.STOP_LINE


@pytest.mark.parametrize("x0, x1", [(85, 235), (40, 270)])
def test_wide_stop_line(x0, x1):
    """긴 정지선 — 7.5배, 11.5배 (화면 끝에 닿지 않으면 정지선)."""
    img = floor()
    vline(img, 160)
    hline(img, x0, x1, STOP_Y)
    assert run(img).event == LineEvent.STOP_LINE


@pytest.mark.parametrize("y", [165, 180, 195, 210, 220])
def test_stop_line_found_anywhere_in_roi(y):
    """먼 띠부터 가까운 띠까지 어디 있어도 잡힘 — 확인 프레임을 채울 시간이 남음 (이전엔 중간 띠에서만)."""
    img = floor()
    vline(img, 160)
    hline(img, 100, 220, (y, y + 15))
    assert run(img).event == LineEvent.STOP_LINE


def test_stop_line_on_tilted_line():
    """기울어진 라인 — 정지선 위치의 라인을 기준으로 양쪽 걸침을 봄."""
    import cv2
    img = floor()
    cv2.line(img, (120, H - 1), (220, 100), (25, 25, 25), LW)
    x_at_y = 120 + (H - 1 - 180) * 100 // (H - 1 - 100)
    hline(img, x_at_y - 60, x_at_y + 60, (170, 190))   # 기운 라인은 가로 폭이 굵어 두께도 넉넉히
    assert run(img).event == LineEvent.STOP_LINE


def test_same_tape_stop_line_foreshortened():
    """라인과 같은 테이프 — 원근으로 세로가 눌려 라인의 0.76배로 보임 (2026-11-02 실제 사진: 라인 24~25px, 가로 테이프 19px)."""
    img = floor()
    vline(img, 160)
    hline(img, 110, 210, (192, 207))      # 두께 15px = 라인 20px의 0.75배
    assert run(img).event == LineEvent.STOP_LINE


@pytest.mark.parametrize("x", [296, 24])
def test_straight_line_near_edge_is_not_corner(x):
    """곧은 라인이 화면 끝에 가까워도 코너 아님 (2026-11-02 실제 사진: x 286~306에서 CORNER_RIGHT 오판정 → 주행 중 정지)."""
    img = floor()
    vline(img, x)
    r = run(img)
    assert r.line_found and r.raw_event == LineEvent.NONE and r.event == LineEvent.NONE


@pytest.mark.parametrize("thick", [6, 11, 12])
def test_thin_floor_line_is_not_stop_line(thick):
    """바닥의 얇은 가로선(줄눈·자국)이 라인을 가로질러도 정지선 아님 (2026-10-30 S1).
    6px은 중간 띠 40% 조건에서, 11·12px은 두께 조건(라인 20px × 0.65 = 13px)에서 걸러짐."""
    img = floor()
    vline(img, 160)
    img[196:196 + thick, 115:205] = 25         # 폭 90px — 이전 기준(2~5배) 안
    r = run(img)
    assert r.raw_event == LineEvent.NONE and r.event == LineEvent.NONE


def test_one_sided_mark_is_not_stop_line():
    """라인 한쪽에만 붙은 두꺼운 자국 — 정지선은 라인 양쪽으로 걸쳐야 함."""
    img = floor()
    vline(img, 160)
    hline(img, 160, 250, STOP_Y)
    assert run(img).raw_event == LineEvent.NONE


def terrazzo():
    """무늬 바닥 — 밝은 바탕(V 약 190)에 회색 조각(V 약 165) (2026-11-02 실제 바닥)."""
    rng = np.random.default_rng(1)
    img = np.full((H, W, 3), 190, np.uint8)
    for _ in range(250):
        x, y = int(rng.integers(0, W)), int(rng.integers(0, H))
        img[y:y + int(rng.integers(3, 9)), x:x + int(rng.integers(3, 12))] = 165
    return img


@pytest.mark.parametrize("v", [40, 90, 124, 135])
def test_glossy_line_on_patterned_floor(v):
    """반사로 밝아진 라인(V 124 실측)도 무늬 바닥에서 찾음 — 고정 문턱 110이면 LINE_END로 멈췄음."""
    img = terrazzo()
    img[:, 150:170] = v
    r = run(img, frames=12)
    assert r.line_found and abs(r.offset) < 0.05 and r.event == LineEvent.NONE


def test_patterned_floor_alone_is_line_end():
    """회색 무늬만 있으면 라인 아님."""
    r = run(terrazzo(), frames=12)
    assert not r.line_found and r.event == LineEvent.LINE_END


def test_line_end():
    r = run(floor(), frames=LineConfig().line_end_frames)
    assert not r.line_found and r.event == LineEvent.LINE_END


def test_line_end_needs_longer_confirmation():
    """잠깐(3~9프레임) 라인을 놓친 것은 라인 끝이 아님 — 2026-10-30 S1: 주행 중 LINE_END로 멈췄다가 다시 출발하면 바로 따라감."""
    det = LineDetector(LineConfig())
    img = floor()
    vline(img, 160)
    for _ in range(3):
        det.process(img)
    events = [det.process(floor()).event for _ in range(9)]       # 9프레임 놓침 (30fps ≈ 0.3초)
    assert all(e == LineEvent.NONE for e in events)
    assert det.process(img).line_found                         # 다시 보이면 이어서
    assert det.process(floor()).event == LineEvent.NONE        # 세는 것도 처음부터


def test_set_frames_applies_immediately():
    det = LineDetector(LineConfig())
    det.set_frames("LINE_END", 4)
    assert [det.process(floor()).event for _ in range(4)][-1] == LineEvent.LINE_END
    det = LineDetector(LineConfig())
    det.set_frames("CONFIRM", 1)
    img = floor()
    vline(img, 160)
    hline(img, 110, 210, STOP_Y)
    assert det.process(img).event == LineEvent.STOP_LINE      # 1프레임에 확정
    with pytest.raises(ValueError):
        det.set_frames("FPS", 5)


def test_line_visible_above_is_not_line_end():
    """가까운 띠에서만 놓치고 위쪽에 라인이 보이면 잠깐 놓친 것 — 라인 끝으로 세지 않음."""
    img = floor()
    vline(img, 200, 0, 205)                       # 가까운 띠(213~)엔 없음
    r = run(img, frames=12)
    assert not r.line_found and r.raw_event == LineEvent.NONE and r.event == LineEvent.NONE


def test_dark_thing_far_from_line_is_not_line_above():
    """직전 라인 위치에서 먼 어두운 물체(화면 가장자리)는 "위쪽 라인"이 아님 — 진짜 라인 끝에서 멈춰야 함 (2026-11-02 실제 사진)."""
    img = floor()
    img[160:205, 0:14] = 40
    r = run(img, frames=LineConfig().line_end_frames)
    assert r.raw_event == LineEvent.LINE_END and r.event == LineEvent.LINE_END


def test_stops_even_if_something_stays_above():
    """가까운 띠에 라인이 계속 없으면 위쪽에 라인 모양이 있어도 결국 라인 끝 — 멈추지 않고 달리는 것 방지."""
    img = floor()
    vline(img, 160, 0, 205)
    det = LineDetector(LineConfig())
    n = LineConfig().line_end_frames
    events = [det.process(img).event for _ in range(3 * n)]
    assert events[n] == LineEvent.NONE and events[-1] == LineEvent.LINE_END


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
    assert abs(r.offset) > 0.1 or r.raw_event != LineEvent.NONE or not r.line_found   # 밀리거나 놓침


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


def test_thick_object_near_center_not_chosen():
    """가운데 가까이 있는 두꺼운 검정 물체(폭 80px, 아래까지 닿음) — 2026-10-30 S1에서 오프셋이 끌려간 경우."""
    img = floor()
    vline(img, 160)
    img[150:240, 55:135] = 30                     # 라인과 15px 떨어짐
    r = run(img)
    assert r.line_found and abs(r.offset) < 0.05


def test_thick_object_touching_line_does_not_pull_offset():
    """라인에 붙은 두꺼운 물체 — 한 덩어리가 돼도 줄마다 가는 구간(라인)만 따라감."""
    img = floor()
    vline(img, 160)
    img[222:240, 170:260] = 30                    # 라인 오른쪽에 붙음 (가까운 띠 아래쪽 절반)
    r = run(img)
    assert r.line_found and abs(r.offset) < 0.05


def test_thick_object_does_not_inflate_line_width():
    """두꺼운 물체를 본 뒤에도 라인 굵기 기준이 커지지 않아 정지선 판정이 유지됨."""
    det = LineDetector(LineConfig())
    img = floor()
    vline(img, 160)
    img[150:240, 55:135] = 30
    for _ in range(3):
        det.process(img)
    stop = floor()
    vline(stop, 160)
    hline(stop, 110, 210, STOP_Y)
    for _ in range(3):
        r = det.process(stop)
    assert r.event == LineEvent.STOP_LINE and det.line_px <= LineConfig().max_line_px


def slanted(x_bottom, x_top, y0=0):
    """아래 x_bottom → 위 x_top으로 기운 라인 (굵기 LW)."""
    import cv2
    img = floor()
    cv2.line(img, (x_bottom, H - 1), (x_top, y0), (25, 25, 25), LW)
    return img


@pytest.mark.parametrize("x_bottom, x_top, want", [
    (160, 160, 90.0),          # 똑바로
    (140, 220, None),          # 위쪽이 오른쪽 → 90° 미만
    (180, 100, None),          # 위쪽이 왼쪽 → 90° 초과
])
def test_line_angle(x_bottom, x_top, want):
    import math
    r = run(slanted(x_bottom, x_top))
    expect = math.degrees(math.atan2(H - 1, x_top - x_bottom)) if want is None else want
    assert r.angle_valid and abs(r.angle - expect) < 2.0


def test_angle_sign_matches_tilt():
    right = run(slanted(140, 220))
    left = run(slanted(180, 100))
    assert right.angle < 85 and left.angle > 95


def test_angle_ignores_corner_horizontal():
    """코너의 가로선은 넓어서 각도 계산에서 빠짐 — 세로 라인 각도 그대로."""
    img = floor()
    vline(img, 160, MID_Y[0], H)
    hline(img, 0, 170)
    r = run(img)
    assert r.angle_valid and abs(r.angle - 90) < 3


def test_angle_invalid_when_too_short():
    img = floor()
    vline(img, 160, 232, H)                       # 아래 8줄만
    r = run(img)
    assert not r.angle_valid and r.angle == 90.0


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

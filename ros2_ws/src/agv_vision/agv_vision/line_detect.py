"""라인 검출 (② 순수 로직) — BGR 영상 → 오프셋·이벤트 (functional_design F4, class_design 2.1·2.2).

처리: 320×240 축소 → 아래쪽 roi_ratio 관심 영역 → 검정 마스크 → 띠 3개
  가까운 띠(아래): 라인 중심 → 오프셋 -1(왼쪽) ~ 1(오른쪽), 라인 굵기 추정
  중간 띠       : 가로 검은 영역 — 폭·화면 양끝 도달 → 정지선·분기점·코너
  먼 띠(위)     : 가운데 쪽 세로 라인 = "앞쪽 라인 있음"
검정 = 어둡고(V가 Otsu 문턱 이하) + 색이 없음(S가 낮음) → 빨강·파랑 마커가 라인으로 잡히지 않게.
같은 이벤트가 confirm_frames번 연속일 때만 확정(EventConfirmer).
"""
from dataclasses import dataclass

import cv2
import numpy as np

from agv_protocol.types import LineEvent


@dataclass
class LineConfig:
    width: int = 320
    height: int = 240
    roi_ratio: float = 0.33             # 아래쪽 몇 %를 볼지
    black_v_range: tuple = (40, 110)    # Otsu 문턱을 이 범위로 제한 (조명 변화·라인 없는 화면 대비)
    black_s_max: int = 90               # 채도가 이보다 크면 색 마커 → 라인 아님
    min_pixels: int = 60                # 가까운 띠에 이보다 적으면 라인 없음
    column_fill: float = 0.4            # 띠 높이의 몇 % 이상 검으면 그 열을 "검은 열"로 봄
    edge_ratio: float = 0.05            # 화면 양끝 몇 % 안에 닿으면 "끝까지"
    stop_line_width_ratio: tuple = (2.0, 5.0)   # 정지선 = 가로 폭이 라인 굵기의 2~5배
    ahead_center_ratio: float = 0.4     # 먼 띠에서 가운데 ±40% 안의 세로 라인 = 앞쪽 라인
    default_line_px: int = 20           # 라인 굵기 추정이 안 될 때
    confirm_frames: int = 3


@dataclass
class LineResult:
    offset: float           # -1.0 ~ 1.0 (라인 중심 좌우 오차, 오른쪽 +)
    line_found: bool
    event: LineEvent        # 확정된 이벤트 (연속 프레임 통과분만)
    raw_event: LineEvent    # 이번 프레임 판정 (확인 전)


class EventConfirmer:
    """같은 이벤트가 frames번 연속이면 그 이벤트, 아니면 NONE."""

    def __init__(self, frames: int = 3):
        self.frames = frames
        self.last = LineEvent.NONE
        self.count = 0

    def push(self, event: LineEvent) -> LineEvent:
        self.count = self.count + 1 if event == self.last else 1
        self.last = event
        return event if event != LineEvent.NONE and self.count >= self.frames else LineEvent.NONE


def black_mask(bgr: np.ndarray, cfg: LineConfig) -> np.ndarray:
    """어둡고 색이 없는 픽셀 = 255."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    s, v = hsv[..., 1], hsv[..., 2]
    otsu, _ = cv2.threshold(v, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    lo, hi = cfg.black_v_range
    thr = min(max(otsu, lo), hi)
    mask = ((v <= thr) & (s <= cfg.black_s_max)).astype(np.uint8) * 255
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def _black_columns(band: np.ndarray, fill: float) -> np.ndarray:
    """띠에서 검은 열 여부 (열마다 검은 비율 ≥ fill)."""
    return (band > 0).mean(axis=0) >= fill


def _spans(cols: np.ndarray) -> list[tuple[int, int]]:
    """연속된 True 구간 [(시작, 끝 포함)]."""
    spans, start = [], None
    for i, on in enumerate(cols):
        if on and start is None:
            start = i
        elif not on and start is not None:
            spans.append((start, i - 1))
            start = None
    if start is not None:
        spans.append((start, len(cols) - 1))
    return spans


class LineDetector:
    def __init__(self, cfg: LineConfig | None = None):
        self.cfg = cfg or LineConfig()
        self.confirm = EventConfirmer(self.cfg.confirm_frames)
        self.offset = 0.0

    def process(self, bgr: np.ndarray) -> LineResult:
        c = self.cfg
        img = cv2.resize(bgr, (c.width, c.height), interpolation=cv2.INTER_AREA)
        roi = img[int(c.height * (1 - c.roi_ratio)):, :]
        mask = black_mask(roi, c)
        h = mask.shape[0]
        far, mid, near = mask[: h // 3], mask[h // 3: 2 * h // 3], mask[2 * h // 3:]

        found, offset, line_px = self.center_offset(near)
        if found:
            self.offset = offset
        raw = self.detect_event(mid, far, near, line_px, found)
        return LineResult(round(self.offset, 3), found, self.confirm.push(raw), raw)

    def center_offset(self, near: np.ndarray) -> tuple[bool, float, float]:
        """(라인 있음, 오프셋, 라인 굵기 px). 가장 큰 덩어리 중 이전 위치에 가까운 것을 쓴다."""
        c = self.cfg
        n, labels, stats, cents = cv2.connectedComponentsWithStats(near)
        best = None
        prev_x = (self.offset + 1) * c.width / 2
        for i in range(1, n):
            area = stats[i, cv2.CC_STAT_AREA]
            if area < c.min_pixels:
                continue
            score = area - abs(cents[i][0] - prev_x)       # 크고 이전 위치에 가까울수록
            if best is None or score > best[0]:
                best = (score, i)
        if best is None:
            return False, self.offset, c.default_line_px
        i = best[1]
        cx = cents[i][0]
        line_px = stats[i, cv2.CC_STAT_AREA] / max(1, stats[i, cv2.CC_STAT_HEIGHT])
        return True, float(np.clip((cx - c.width / 2) / (c.width / 2), -1.0, 1.0)), float(line_px)

    def detect_event(self, mid, far, near, line_px, near_found) -> LineEvent:
        """F4 판단표 — 중간 띠의 가로 폭·양끝 도달, 먼 띠의 앞쪽 라인."""
        c = self.cfg
        w = mid.shape[1]
        edge = max(1, int(w * c.edge_ratio))
        if not near_found and not (mid > 0).any() and not (far > 0).any():
            return LineEvent.LINE_END
        spans = _spans(_black_columns(mid, c.column_fill))
        if not spans:
            return LineEvent.NONE
        widest = max(spans, key=lambda s: s[1] - s[0])
        width = widest[1] - widest[0] + 1
        left, right = widest[0] < edge, widest[1] >= w - edge
        ahead = any(abs((a + b) / 2 - w / 2) <= w * c.ahead_center_ratio and (b - a + 1) < 3 * line_px
                    for a, b in _spans(_black_columns(far, c.column_fill)))
        lo, hi = c.stop_line_width_ratio
        if left and right:
            return LineEvent.NONE if ahead else LineEvent.JUNCTION_T      # 십자는 트랙에 없음
        if left:
            return LineEvent.JUNCTION_LEFT if ahead else LineEvent.CORNER_LEFT
        if right:
            return LineEvent.JUNCTION_RIGHT if ahead else LineEvent.CORNER_RIGHT
        if lo * line_px <= width <= hi * line_px:
            return LineEvent.STOP_LINE
        return LineEvent.NONE

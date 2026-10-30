"""라인 검출 (② 순수 로직) — BGR 영상 → 오프셋·이벤트 (functional_design F4, class_design 2.1·2.2).

처리: 320×240 축소 → 아래쪽 roi_ratio 관심 영역 → 검정 마스크 → 연결 성분 → 띠 3개
  라인 고르기  : 가까운 띠에 걸친 덩어리 중 점수 = 면적(행마다 라인 굵기 2배까지만) × 가운데 가중치 × 직전 위치 가중치
                × (화면 맨 아래에 닿으면 가산)
                → 주변의 어두운 물체보다 차량 밑에서 들어오는 가운데 라인을 고른다
  가까운 띠(아래): 고른 라인의 중심 → 오프셋 -1(왼쪽) ~ 1(오른쪽), 라인 굵기 추정
  중간 띠       : 고른 라인과 **이어진** 가로 검은 영역 — 폭·화면 양끝 도달 → 정지선·코너
  먼 띠(위)     : 고른 라인과 이어진 가운데 쪽 세로 라인 = "앞쪽 라인 있음"
  (가중치는 라인 고르기에만 — 코너 라인은 화면 끝까지 뻗어야 하므로 이벤트는 "이어져 있는지"로 거른다)
  갈림길(분기점)은 판정하지 않는다 → 색 마커 인식이 맡음 (2026-10-30 결정). T자·옆 갈래 모양은 NONE,
  마커를 놓쳐 T자에서 라인이 끊기면 LINE_END로 멈춘다.
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
    center_weight_min: float = 0.3      # 라인 고르기 가중치: 가운데 1.0 → 화면 양끝 이 값
    prev_weight_min: float = 0.5        # 직전 라인 위치에서 화면 폭 절반만큼 떨어지면 이 값
    bottom_bonus: float = 1.5           # 화면 맨 아래(차량 바로 앞)에 닿는 덩어리 가산
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
        self.line_px = float(self.cfg.default_line_px)     # 직전 라인 굵기 (넓은 물체 면적 제한에 사용)

    def process(self, bgr: np.ndarray) -> LineResult:
        c = self.cfg
        img = cv2.resize(bgr, (c.width, c.height), interpolation=cv2.INTER_AREA)
        roi = img[int(c.height * (1 - c.roi_ratio)):, :]
        mask = black_mask(roi, c)
        h = mask.shape[0]
        _, labels = cv2.connectedComponents(mask, connectivity=8)
        line_id = self.select_line(labels, h)
        if line_id is None:                                   # 주변 물체만 있거나 아무것도 없음
            raw = LineEvent.LINE_END
            return LineResult(round(self.offset, 3), False, self.confirm.push(raw), raw)
        line = np.where(labels == line_id, 255, 0).astype(np.uint8)   # 고른 라인 + 이어진 갈래만
        far, mid, near = line[: h // 3], line[h // 3: 2 * h // 3], line[2 * h // 3:]
        self.offset, self.line_px = self.center_offset(near)
        raw = self.detect_event(mid, far, self.line_px)
        return LineResult(round(self.offset, 3), True, self.confirm.push(raw), raw)

    def select_line(self, labels: np.ndarray, h: int) -> int | None:
        """가까운 띠에 걸친 덩어리 중 라인 하나. 점수 = 면적(행마다 라인 굵기 2배까지만) × 가운데 가중치
        × 직전 위치 가중치 × 아래 닿음 가산 — 넓은 어두운 물체가 면적만으로 이기지 못하게."""
        c = self.cfg
        near = labels[2 * h // 3:]
        half = c.width / 2
        prev_x = (self.offset + 1) * half
        cap = 2 * max(self.line_px, c.default_line_px)
        best = None
        for i in np.unique(near):
            if i == 0:
                continue
            ys, xs = np.nonzero(near == i)
            if len(xs) < c.min_pixels:
                continue
            cx = xs.mean()
            w_center = 1.0 - (1.0 - c.center_weight_min) * min(1.0, abs(cx - half) / half)
            w_prev = 1.0 - (1.0 - c.prev_weight_min) * min(1.0, abs(cx - prev_x) / half)
            bonus = c.bottom_bonus if ys.max() >= near.shape[0] - 2 else 1.0
            area = np.minimum(np.bincount(ys), cap).sum()          # 행마다 cap까지만 셈
            score = area * w_center * w_prev * bonus
            if best is None or score > best[0]:
                best = (score, int(i))
        return None if best is None else best[1]

    def center_offset(self, near: np.ndarray) -> tuple[float, float]:
        """고른 라인(가까운 띠)의 (오프셋, 라인 굵기 px)."""
        c = self.cfg
        ys, xs = np.nonzero(near)
        rows = max(1, len(np.unique(ys)))
        offset = float(np.clip((xs.mean() - c.width / 2) / (c.width / 2), -1.0, 1.0))
        return offset, len(xs) / rows

    def detect_event(self, mid, far, line_px) -> LineEvent:
        """F4 판단표 — 고른 라인과 이어진 영역에서 중간 띠의 가로 폭·양끝 도달, 먼 띠의 앞쪽 라인.
        갈림길은 마커 인식이 맡으므로 T자·옆 갈래 모양은 NONE."""
        c = self.cfg
        w = mid.shape[1]
        edge = max(1, int(w * c.edge_ratio))
        spans = _spans(_black_columns(mid, c.column_fill))
        if not spans:
            return LineEvent.NONE
        widest = max(spans, key=lambda s: s[1] - s[0])
        width = widest[1] - widest[0] + 1
        left, right = widest[0] < edge, widest[1] >= w - edge
        ahead = any(abs((a + b) / 2 - w / 2) <= w * c.ahead_center_ratio and (b - a + 1) < 3 * line_px
                    for a, b in _spans(_black_columns(far, c.column_fill)))
        lo, hi = c.stop_line_width_ratio
        if left and right or (left or right) and ahead:        # T자·옆 갈래 = 갈림길 → 마커 인식
            return LineEvent.NONE
        if left:
            return LineEvent.CORNER_LEFT
        if right:
            return LineEvent.CORNER_RIGHT
        if lo * line_px <= width <= hi * line_px:
            return LineEvent.STOP_LINE
        return LineEvent.NONE

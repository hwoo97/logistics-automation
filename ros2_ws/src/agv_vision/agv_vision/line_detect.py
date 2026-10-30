"""라인 검출 (② 순수 로직) — BGR 영상 → 오프셋·이벤트 (functional_design F4, class_design 2.1·2.2).

처리: 320×240 축소 → 아래쪽 roi_ratio 관심 영역 → 검정 마스크 → 연결 성분 → 띠 3개
  라인 고르기  : 줄마다 "라인만큼 가는" 구간(폭 ≤ max_line_px)만 라인 후보로 센다 — 두꺼운 물체는 붙어 있든
                떨어져 있든 후보가 아님. 덩어리 점수 = 가는 구간 면적 × 가운데 가중치 × 직전 위치 가중치
                × (화면 맨 아래에 닿으면 가산)
  가까운 띠(아래): 고른 라인에서 줄마다 직전 위치에 가장 가까운 가는 구간의 중심 평균 → 오프셋 -1(왼쪽) ~ 1(오른쪽),
                그 폭의 중앙값 → 라인 굵기 (max_line_px로 제한해 물체 폭이 기준이 되지 않게)
  라인 각도    : 관심 영역 맨 아래 줄부터 위로, 줄마다 바로 아래 줄 위치에 가까운 가는 구간을 따라가 중심점을 모으고
                직선 x = a·y + b로 맞춰 각도 계산 — 90° = 화면 위로 똑바로, 90° 미만 = 위쪽이 오른쪽으로 기움
                (두 끝점만 쓰면 끝부분 흔들림에 민감해서 직선 맞춤)
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
    stop_line_width_ratio: tuple = (2.0, 8.0)   # 정지선 = 가로 폭이 라인 굵기의 2~8배 (10cm ÷ 19mm ≈ 5.3배라 5배에서 늘림)
    stop_min_thickness: float = 0.8     # 정지선 세로 두께 ≥ 라인 굵기 × 이 값 — 바닥의 얇은 줄눈·자국 제외
                                        #   (정지선은 40~50mm 테이프로 라인 19mm보다 두꺼움, 원근으로 세로는 줄어 보임)
    ahead_center_ratio: float = 0.4     # 먼 띠에서 가운데 ±40% 안의 세로 라인 = 앞쪽 라인
    default_line_px: int = 20           # 라인 굵기 추정이 안 될 때
    max_line_px: int = 45               # 한 줄에서 이보다 넓은 검은 구간은 라인이 아님 (두꺼운 물체, 붙은 물체)
    angle_min_rows: int = 15            # 각도를 믿으려면 라인을 따라간 줄이 이만큼은 있어야 함
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
    angle: float = 90.0     # 라인 각도(도) — 90 = 똑바로, 90 미만 = 위쪽이 오른쪽으로 기움
    angle_valid: bool = False


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
        self.line_px = float(self.cfg.default_line_px)     # 직전 라인 굵기 (정지선 폭 기준)

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
        near = line[2 * h // 3:]                                # 오프셋은 가까운 띠, 이벤트는 detect_event가 전체에서
        centered = self.center_offset(near)
        if centered is None:                                  # 가까운 띠가 전부 두꺼움 (물체에 덮임)
            raw = LineEvent.LINE_END
            return LineResult(round(self.offset, 3), False, self.confirm.push(raw), raw)
        self.offset, self.line_px = centered
        raw = self.detect_event(line, self.line_px)
        angle = self.line_angle(line)
        return LineResult(round(self.offset, 3), True, self.confirm.push(raw), raw,
                          round(angle, 1) if angle is not None else 90.0, angle is not None)

    def line_angle(self, line: np.ndarray) -> float | None:
        """관심 영역 전체에서 라인을 아래→위로 따라가 직선을 맞춘 각도(도). 줄이 부족하면 None.
        줄마다 바로 아래 중심에서 max_line_px 안쪽의 가는 구간만 이어 붙임 — 코너·정지선 가로선은 넓어서 빠짐."""
        c = self.cfg
        ref = (self.offset + 1) * c.width / 2
        ys, xs = [], []
        for y in range(line.shape[0] - 1, -1, -1):
            runs = self._thin_runs(line[y] > 0, ref)
            if runs:
                a, b = runs[0]
                x = (a + b) / 2
                if abs(x - ref) > c.max_line_px:                 # 다른 덩어리로 건너뜀 방지
                    continue
                ref = x
                ys.append(y)
                xs.append(x)
        if len(ys) < c.angle_min_rows or max(ys) - min(ys) < c.angle_min_rows:
            return None
        a, _ = np.polyfit(np.array(ys, float), np.array(xs, float), 1)    # x = a·y + b (y는 아래로 증가)
        return float(np.degrees(np.arctan2(1.0, -a)))                     # 위로 한 줄 갈 때 x가 −a만큼

    def _thin_runs(self, row: np.ndarray, ref_x: float):
        """한 줄의 검은 구간 중 폭 ≤ max_line_px 인 것, ref_x에 가까운 순."""
        runs = [(a, b) for a, b in _spans(row) if b - a + 1 <= self.cfg.max_line_px]
        return sorted(runs, key=lambda r: abs((r[0] + r[1]) / 2 - ref_x))

    def select_line(self, labels: np.ndarray, h: int) -> int | None:
        """가까운 띠에 걸친 덩어리 중 라인 하나. 점수 = 가는 구간(폭 ≤ max_line_px) 면적 × 가운데 가중치
        × 직전 위치 가중치 × 아래 닿음 가산 — 두꺼운 물체는 가는 구간이 없어 후보가 되지 못함."""
        c = self.cfg
        near = labels[2 * h // 3:]
        half = c.width / 2
        prev_x = (self.offset + 1) * half
        best = None
        for i in np.unique(near):
            if i == 0:
                continue
            comp = near == i
            if comp.sum() < c.min_pixels:
                continue
            widths, centers, bottom = [], [], False
            for y in range(comp.shape[0]):
                runs = self._thin_runs(comp[y], prev_x)
                if runs:
                    a, b = runs[0]
                    widths.append(b - a + 1)
                    centers.append((a + b) / 2)
                    bottom |= y >= comp.shape[0] - 2
            area = sum(widths)
            if area < c.min_pixels / 2:
                continue
            cx = float(np.mean(centers))
            w_center = 1.0 - (1.0 - c.center_weight_min) * min(1.0, abs(cx - half) / half)
            w_prev = 1.0 - (1.0 - c.prev_weight_min) * min(1.0, abs(cx - prev_x) / half)
            bonus = c.bottom_bonus if bottom else 1.0
            score = area * w_center * w_prev * bonus
            if best is None or score > best[0]:
                best = (score, int(i))
        return None if best is None else best[1]

    def center_offset(self, near: np.ndarray) -> tuple[float, float] | None:
        """고른 라인(가까운 띠)의 (오프셋, 라인 굵기 px). 아래 줄부터 위로, 줄마다 바로 아래 줄 위치에 가장 가까운
        가는 구간을 따라간다 — 붙어 있는 두꺼운 물체 줄은 건너뜀. 가는 구간이 하나도 없으면 None."""
        c = self.cfg
        ref = (self.offset + 1) * c.width / 2
        centers, widths = [], []
        for y in range(near.shape[0] - 1, -1, -1):
            runs = self._thin_runs(near[y] > 0, ref)
            if runs:
                a, b = runs[0]
                ref = (a + b) / 2
                centers.append(ref)
                widths.append(b - a + 1)
        if not centers:
            return None
        offset = float(np.clip((np.mean(centers) - c.width / 2) / (c.width / 2), -1.0, 1.0))
        return offset, float(np.median(widths))

    def detect_event(self, line: np.ndarray, line_px: float) -> LineEvent:
        """F4 판단표 — 고른 라인과 이어진 영역에서 중간 띠의 가로 폭·양끝 도달, 먼 띠의 앞쪽 라인.
        갈림길은 마커 인식이 맡으므로 T자·옆 갈래 모양은 NONE."""
        c = self.cfg
        h = line.shape[0]
        far, mid = line[: h // 3], line[h // 3: 2 * h // 3]
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
        if lo * line_px <= width <= hi * line_px and self._is_stop_line(line, widest, line_px):
            return LineEvent.STOP_LINE
        return LineEvent.NONE

    def _is_stop_line(self, line: np.ndarray, span: tuple, line_px: float) -> bool:
        """가로 검은 영역이 정지선인지 — ① 라인의 왼쪽·오른쪽에 모두 걸침 ② 라인 바깥 부분의 세로 두께가
        라인 굵기 × stop_min_thickness 이상 (2026-10-30 S1: 바닥의 얇은 가로선이 정지선으로 잡힘)."""
        c = self.cfg
        a, b = span
        line_x = (self.offset + 1) * c.width / 2
        keep_out = 1.5 * line_px                                   # 세로 라인이 지나가는 열은 두께에서 뺌
        if a > line_x - keep_out or b < line_x + keep_out:          # 한쪽에만 붙은 자국
            return False
        cols = [x for x in range(a, b + 1) if abs(x - line_x) > keep_out]
        thickness = float(np.median((line[:, cols] > 0).sum(axis=0)))
        return thickness >= c.stop_min_thickness * line_px

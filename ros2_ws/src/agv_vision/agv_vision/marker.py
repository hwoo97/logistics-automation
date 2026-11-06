"""색 마커 판별 (② 순수 로직) — BGR 영상 + 라인 위치 → 색 스티커의 색·위치(left/center/right)·화면 높이 (functional_design F6).

마커 = 지름 약 2cm 원형 스티커를 **라인 테이프 위**에 붙임 (2026-11-06 변경, 이전: 라인 옆 좌우).
  center = 앞으로 이어지는 라인(들어오는 라인·분기점 너머 직진 라인) 위 → 목적지 색이면 직진
  left / right = 옆 갈래 라인 위 → 목적지 색이면 그쪽으로 회전
판별: 스티커 바로 아래(차 쪽, below_from~below_to × 지름)에 검은 라인이 below_black 이상이면 center
  (앞으로 뻗은 라인 위 스티커는 아래로 라인이 이어지고, 옆 갈래 위 스티커는 아래가 바닥).
  아래가 화면 밖이면 라인 위치와의 거리(center_ratio × 지름 안)로. center가 아니면 라인 기준 좌우.
어느 색을 따라갈지·언제 멈출지는 agv_control.drive가 정한다 — 여기서는 보이는 것만 알려 준다.
좌우는 화면 가운데가 아니라 라인 기준 (카메라가 차체 중심에서 비켜 있음). 라인이 기울면 스티커 높이의 라인 위치로 비교.
"""
from dataclasses import dataclass, fields

import cv2
import numpy as np

COLORS = ("red", "yellow", "green", "blue")


@dataclass
class MarkerConfig:
    width: int = 320
    height: int = 240
    # 색별 (H 시작, H 끝, S 최소, V 최소). 시작 > 끝이면 0을 넘어가는 범위 (2026-11-02 실제 스티커 사진)
    red: tuple = (168, 10, 125, 125)        # 실측 H174~5 S140~200 V145~170 — S·V 110·100이면 손의 붉은 점이 잡힘
    yellow: tuple = (24, 38, 120, 120)      # 실측 H28~35 S200~240 V185~200
    green: tuple = (60, 85, 90, 70)         # 실측 H67~77 S130~175 V115~135 — 가장 약함
    blue: tuple = (100, 116, 120, 90)       # 실측 H104~111 S215~250 V155~180
    open_kernel: int = 3                    # 잡음 제거 — 크면 먼 스티커가 지워짐
    min_area: int = 30                      # 먼 곳(y45) 스티커 약 57px
    min_fill: float = 0.5                   # 면적 ÷ 외접 사각형 (원 0.79, 실측 0.72~0.85)
    size_base: float = 8.5                  # 높이 y에서 스티커 지름(px) ≈ size_base + size_slope × y (실측 y45 12, y215 25)
    size_slope: float = 0.076
    width_ratio: tuple = (0.5, 1.6)         # 가로 폭 ÷ 예상 지름 — 얼룩과 큰 색 물체 제외
    max_height_ratio: float = 2.0           # 세로 ÷ 예상 지름 — 주행 중 세로로 번져도 인정
    line_y: float = 226.0                   # /line/state 오프셋을 잰 높이 (가까운 띠의 가운데)
    below_from: float = 0.8                 # 스티커 아래 검사 창: 중심에서 지름 × 0.8 ~ 1.8 아래, 좌우 ± 지름 × 0.3
    below_to: float = 1.8
    below_half_width: float = 0.3
    below_black: float = 0.5                # 창의 이 비율 이상이 검정이면 아래로 라인이 이어짐 → center
    center_ratio: float = 1.0               # 창이 화면 밖일 때: 라인 위치에서 지름 × 이 안이면 center
    black_ratio: float = 0.72               # 검정 = 바닥 밝기(화면 상위 10%) × 이 비율 이하, 채도 낮음 (line_detect와 같은 기준)
    black_v_max: int = 170
    black_s_max: int = 90


@dataclass
class Marker:
    color: str
    side: str       # left / center / right — center = 앞으로 이어지는 라인 위, left/right = 옆 갈래 위
    x: float
    y: float        # 화면 높이 — 클수록 차에 가까움
    area: int


class MarkerDetector:
    def __init__(self, cfg: MarkerConfig | None = None):
        self.cfg = cfg or MarkerConfig()
        self.kernel = np.ones((self.cfg.open_kernel, self.cfg.open_kernel), np.uint8)

    def _mask(self, hsv: np.ndarray, spec: tuple) -> np.ndarray:
        h_lo, h_hi, s_min, v_min = spec
        if h_lo <= h_hi:
            mask = cv2.inRange(hsv, (h_lo, s_min, v_min), (h_hi, 255, 255))
        else:
            mask = (cv2.inRange(hsv, (h_lo, s_min, v_min), (179, 255, 255))
                    | cv2.inRange(hsv, (0, s_min, v_min), (h_hi, 255, 255)))
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

    def _is_sticker(self, y: float, w: int, h: int, area: int) -> bool:
        c = self.cfg
        size = c.size_base + c.size_slope * y
        return (area >= c.min_area and area / (w * h) >= c.min_fill
                and c.width_ratio[0] * size <= w <= c.width_ratio[1] * size and h <= c.max_height_ratio * size)

    def _line_x(self, y: float, offset: float, angle: float | None) -> float:
        c = self.cfg
        x = (offset + 1) * c.width / 2
        if angle is not None and 30.0 <= angle <= 150.0:
            x += (c.line_y - y) / np.tan(np.radians(angle))
        return x

    def _black(self, hsv: np.ndarray) -> np.ndarray:
        c = self.cfg
        v, s = hsv[..., 2], hsv[..., 1]
        floor = float(np.percentile(v, 90))
        return (v <= c.black_ratio * floor) & (v <= c.black_v_max) & (s <= c.black_s_max)

    def _side(self, black: np.ndarray, x: float, y: float, offset: float, angle: float | None) -> str:
        c = self.cfg
        d = c.size_base + c.size_slope * y
        line_x = self._line_x(y, offset, angle)
        y0, y1 = int(round(y + c.below_from * d)), int(round(y + c.below_to * d))
        x0, x1 = int(round(x - c.below_half_width * d)), int(round(x + c.below_half_width * d)) + 1
        if y1 <= black.shape[0] and x0 >= 0 and x1 <= black.shape[1] and y1 > y0:
            if black[y0:y1, x0:x1].mean() >= c.below_black:
                return "center"                                  # 아래로 라인이 이어짐
        elif abs(x - line_x) <= c.center_ratio * d:              # 아래가 화면 밖(바로 앞) — 라인 위치로
            return "center"
        return "left" if x < line_x else "right"

    def process(self, bgr: np.ndarray, offset: float, angle: float | None = None) -> list[Marker]:
        """offset: 라인 오프셋(-1~1), angle: 라인 각도(도, 90 = 똑바로, 못 믿으면 None)."""
        c = self.cfg
        img = cv2.resize(bgr, (c.width, c.height), interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        black = self._black(hsv)
        out = []
        for color in COLORS:
            n, _, stats, centers = cv2.connectedComponentsWithStats(self._mask(hsv, getattr(c, color)), connectivity=8)
            for i in range(1, n):
                _, _, w, h, area = stats[i]
                x, y = centers[i]
                if self._is_sticker(y, w, h, area):
                    side = self._side(black, x, y, offset, angle)
                    out.append(Marker(color, side, round(float(x), 1), round(float(y), 1), int(area)))
        return out


def config_fields() -> list[str]:
    """ROS 파라미터로 내놓을 항목."""
    return [f.name for f in fields(MarkerConfig)]

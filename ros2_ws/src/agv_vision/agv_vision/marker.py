"""색 마커 판별 (② 순수 로직) — BGR 영상 + 라인 위치 → 색 스티커의 색·라인 기준 좌우·화면 높이 (functional_design F6).

마커 = 지름 약 2cm 원형 스티커 (2026-11-02). 목적지 색이 라인 왼쪽만 = 좌회전, 오른쪽만 = 우회전, 양쪽 = 직진.
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


@dataclass
class Marker:
    color: str
    side: str       # left / right (라인 기준)
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

    def process(self, bgr: np.ndarray, offset: float, angle: float | None = None) -> list[Marker]:
        """offset: 라인 오프셋(-1~1), angle: 라인 각도(도, 90 = 똑바로, 못 믿으면 None)."""
        c = self.cfg
        img = cv2.resize(bgr, (c.width, c.height), interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        out = []
        for color in COLORS:
            n, _, stats, centers = cv2.connectedComponentsWithStats(self._mask(hsv, getattr(c, color)), connectivity=8)
            for i in range(1, n):
                _, _, w, h, area = stats[i]
                x, y = centers[i]
                if self._is_sticker(y, w, h, area):
                    side = "left" if x < self._line_x(y, offset, angle) else "right"
                    out.append(Marker(color, side, round(float(x), 1), round(float(y), 1), int(area)))
        return out


def config_fields() -> list[str]:
    """ROS 파라미터로 내놓을 항목."""
    return [f.name for f in fields(MarkerConfig)]

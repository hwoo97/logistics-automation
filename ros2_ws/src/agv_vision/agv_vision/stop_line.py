"""정지선 판정 (② 순수 로직) — BGR 영상 + 라인 오프셋 → 정지선 있음·확정 (functional_design F4).

세로 열 방식: 라인 왼쪽·오른쪽의 열마다 검은 세로 구간을 본다. 정지선이 기울어도 열의 두께는 거의 그대로라
각도에 강하다 (2026-11-02 실제 사진: 가로 줄 방식은 +5°부터 대부분, ±15°부터 전부 놓침).

정지선 조건 — 라인과 이어진 덩어리에서
  열마다  : 가장 긴 세로 구간이 두께 ≥ 라인 굵기 × min_thickness, 관심 영역 맨 위·맨 아래에 닿지 않음
            (위에 닿으면 아직 들어오는 중, 아래에 닿으면 곡선의 대각선 라인일 수 있음)
  라인 위치: 위나 아래에 닿는 열 묶음이 곧 라인 — 정지선이 가까운 띠에 들어오면 오프셋이 끌려가므로 오프셋은 덩어리 고르는 데만 씀
            단 판정 직후에는 아래로 나가는 구간도 인정 — 지연·관성으로 지나쳐 멈춰도 판정이 사라지지 않게
  전체    : 그런 열이 라인 왼쪽·오른쪽에 모두 이어지고, 폭이 라인 굵기의 width_ratio배, 화면 양끝에 모두 닿지 않음(T자 제외),
            기울기 ≤ max_tilt_deg
검정 추출은 line_detect.black_mask를 그대로 쓴다. 같은 판정이 confirm_frames번 쌓이면 확정(한 프레임 빠짐 허용).
"""
from dataclasses import dataclass

import cv2
import numpy as np

from agv_protocol.types import LineEvent
from agv_vision.line_detect import EventConfirmer, _spans, black_mask


@dataclass
class StopLineConfig:
    width: int = 320
    height: int = 240
    roi_ratio: float = 0.5              # 라인(1/3)보다 넓게 — 일찍 잡아야 지연·관성으로 지나쳐도 영역 안에서 멈춤
    black_ratio: float = 0.72           # 이하 4개는 line_detect.LineConfig와 같은 뜻
    black_v_max: int = 170
    black_s_max: int = 90
    black_blur: int = 5
    default_line_px: int = 20
    max_line_px: int = 45
    width_ratio: tuple = (3.0, 12.0)    # 정지선 가로 폭 = 라인 굵기의 3~12배 (트랙 규격 4~5배)
    min_thickness: float = 0.65         # 세로 두께 ≥ 라인 굵기 × 이 값 (실측: 같은 테이프 0.76~0.79배, 얇은 바닥 선 제외)
    side_min_ratio: float = 0.5         # 라인 바깥으로 양쪽 각각 라인 굵기 × 이 값 이상 이어져야 함
    gap_cols: int = 3                   # 정지선 열이 이만큼까지 끊겨도 이어진 것으로 봄 (테이프 반사)
    max_tilt_deg: float = 35.0          # 이보다 기울면 정지선 아님 (대각선 라인)
    edge_ratio: float = 0.05            # 화면 양끝 몇 % 안에 닿으면 "끝까지"
    confirm_frames: int = 3
    hold_frames: int = 20               # 판정 후 이 프레임 동안은 관심 영역 아래로 나가는 정지선도 인정 (지연·관성으로 지나쳐 멈춤)
    hold_max_thick: float = 2.5         # 그때 아래에 닿는 구간 두께 상한 (라인 굵기 배수) — 넘으면 라인으로 봄


@dataclass
class StopLineResult:
    detected: bool          # 이번 프레임 판정
    confirmed: bool         # confirm_frames 쌓여 확정
    angle: float = 0.0      # 정지선 기울기(도), 0 = 수평, + = 오른쪽이 위


def _longest_run(col: np.ndarray) -> tuple[int, int] | None:
    """세로 한 열에서 가장 긴 True 구간 (시작, 끝 포함)."""
    spans = _spans(col)
    return max(spans, key=lambda s: s[1] - s[0]) if spans else None


class StopLineDetector:
    def __init__(self, cfg: StopLineConfig | None = None):
        self.cfg = cfg or StopLineConfig()
        self.confirm = EventConfirmer(self.cfg.confirm_frames, tolerant={LineEvent.STOP_LINE})
        self.hold = 0                                 # 남은 "아래로 나가도 인정" 프레임

    def set_frames(self, frames: int) -> None:
        """노트북 앱 "확인 프레임" (DEV TUNE CONFIRM)."""
        self.cfg.confirm_frames = self.confirm.frames = int(frames)

    def process(self, bgr: np.ndarray, offset: float) -> StopLineResult:
        """offset: line_node의 라인 오프셋 (-1~1)."""
        c = self.cfg
        img = cv2.resize(bgr, (c.width, c.height), interpolation=cv2.INTER_AREA)
        mask = black_mask(img[int(c.height * (1 - c.roi_ratio)):, :], c)
        found, angle = self._find(mask, (offset + 1) * c.width / 2, self.hold > 0)
        self.hold = c.hold_frames if found else max(0, self.hold - 1)
        event = LineEvent.STOP_LINE if found else LineEvent.NONE
        confirmed = self.confirm.push(event) == LineEvent.STOP_LINE
        return StopLineResult(found, confirmed, round(angle, 1))

    def _line_px(self, mask: np.ndarray, line_x: float) -> float:
        """줄마다 라인 위치에 가장 가까운 가는 구간 폭의 중앙값 (정지선이 가까운 띠를 가려도 되게 관심 영역 전체)."""
        c = self.cfg
        widths = []
        for y in range(mask.shape[0]):
            runs = [(a, b) for a, b in _spans(mask[y] > 0) if b - a + 1 <= c.max_line_px]
            if runs:
                a, b = min(runs, key=lambda r: abs((r[0] + r[1]) / 2 - line_x))
                if abs((a + b) / 2 - line_x) <= c.max_line_px:
                    widths.append(b - a + 1)
        return float(np.median(widths)) if widths else float(c.default_line_px)

    def _find(self, mask: np.ndarray, line_x: float, holding: bool = False) -> tuple[bool, float]:
        c = self.cfg
        h, w = mask.shape
        line_px = self._line_px(mask, line_x)
        # 라인과 이어진 덩어리 — 라인 위치 근처 픽셀이 가장 많은 것
        _, labels = cv2.connectedComponents(mask, connectivity=8)
        band = labels[:, max(0, int(line_x - line_px)): int(line_x + line_px) + 1]
        ids, counts = np.unique(band[band > 0], return_counts=True)
        if len(ids) == 0:
            return False, 0.0
        comp = labels == ids[np.argmax(counts)]

        need = max(2.0, c.min_thickness * line_px)
        centers = np.full(w, np.nan)                  # 정지선 열의 세로 중심 (아니면 nan)
        touch = np.zeros(w, bool)                     # 관심 영역 위·아래에 닿는 열 = 라인
        for x in range(w):
            run = _longest_run(comp[:, x])
            if not run:
                continue
            thick = run[1] - run[0] + 1
            if run[0] == 0 or (run[1] == h - 1 and not (holding and thick <= c.hold_max_thick * line_px)):
                touch[x] = True
            elif thick >= need:
                centers[x] = (run[0] + run[1]) / 2

        blocks = _spans(touch)
        if not blocks and not holding:
            return False, 0.0
        if not blocks:                                # 라인이 정지선에서 끝나고 아래로 나가는 중 — 오프셋 위치를 라인으로
            blocks = [(int(line_x - line_px / 2), int(line_x + line_px / 2))]
        merged = [list(blocks[0])]                    # 기운 라인은 정지선 위·아래 부분이 따로 묶임 → 가까우면 하나로
        for a, b in blocks[1:]:
            if a - merged[-1][1] <= line_px:
                merged[-1][1] = b
            else:
                merged.append([a, b])
        line_l, line_r = min(merged, key=lambda s: abs((s[0] + s[1]) / 2 - line_x))
        left = self._extent(centers, line_l - 1, -1, line_px)
        right = self._extent(centers, line_r + 1, 1, line_px)
        if left is None or right is None:
            return False, 0.0
        side = c.side_min_ratio * line_px
        if line_l - left < side or right - line_r < side:
            return False, 0.0
        lo, hi = c.width_ratio
        if not lo * line_px <= right - left + 1 <= hi * line_px:
            return False, 0.0
        edge = max(1, int(w * c.edge_ratio))
        if left < edge and right >= w - edge:         # 양끝까지 = T자 → 마커 인식
            return False, 0.0
        xs = np.nonzero(~np.isnan(centers[left: right + 1]))[0] + left
        slope = np.polyfit(xs, centers[xs], 1)[0]     # 세로 중심 = slope·x + b (y는 아래로 증가)
        angle = float(np.degrees(np.arctan(-slope)))
        return abs(angle) <= c.max_tilt_deg, angle

    def _extent(self, centers: np.ndarray, start: int, step: int, line_px: float) -> int | None:
        """start에서 step 방향으로 정지선 열이 이어지는 가장 먼 x. 첫 열까지는 라인 굵기만큼 비어도 됨(기운 라인)."""
        c = self.cfg
        last, gap = None, 0
        x = start
        while 0 <= x < len(centers):
            if np.isnan(centers[x]):
                gap += 1
                if gap > (c.gap_cols if last is not None else line_px):
                    break
            else:
                last, gap = x, 0
            x += step
        return last

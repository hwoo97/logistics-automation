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
  정지선은 stop_line.py가 판정한다 (2026-11-02 분리, 노드는 line_node 하나).
  중간 띠       : 고른 라인과 **이어진** 가로 검은 영역 — 폭·화면 양끝 도달 → 코너
  먼 띠(위)     : 고른 라인과 이어진 가운데 쪽 세로 라인 = "앞쪽 라인 있음"
  (가중치는 라인 고르기에만 — 코너 라인은 화면 끝까지 뻗어야 하므로 이벤트는 "이어져 있는지"로 거른다)
  옆 갈래(branch_left/right): 관심 영역보다 위(branch_top_ratio)까지 넓혀, 고른 라인과 이어진 가로 검은 구간이
                라인 중심에서 왼쪽·오른쪽으로 branch_reach(화면 폭 비율) 이상 뻗은 줄이 branch_rows줄 이상이면 그쪽 갈래 있음.
                마커 앞에 멈춰 확인할 때 drive가 씀 — 스티커 쪽에 갈래가 없으면 회전하지 않고 직진 (2026-11-06)
  갈림길(분기점)은 판정하지 않는다 → 색 마커 인식이 맡음 (2026-10-30 결정). T자·옆 갈래 모양은 NONE,
  마커를 놓쳐 T자에서 라인이 끊기면 LINE_END로 멈춘다.
검정 = 줄의 바닥 밝기보다 충분히 어둡고 + 색이 없음(S가 낮음) → 반사·그림자에 강함.
색 스티커(채도 높음)는 sticker_as_line이면 라인에 더한다 — 마커를 라인 테이프 위에 붙이므로(2026-11-06) 빼면 라인에 구멍이 남.
  라인과 떨어진 색 물체는 따로 떨어진 작은 덩어리라 라인으로 골라지지 않음.
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
    black_ratio: float = 0.72           # 줄의 바닥 밝기(상위 10%)의 이 비율 이하 = 어두움 (실측: 라인 0.65, 회색 무늬 0.85 이상)
    black_v_max: int = 170              # 이보다 밝으면 대비와 상관없이 라인 아님
    black_blur: int = 5                 # 밝기 중앙값 필터 크기 (홀수)
    black_s_max: int = 90               # 채도가 이보다 크면 색 마커 → 라인 아님
    min_pixels: int = 60                # 가까운 띠에 이보다 적으면 라인 없음
    column_fill: float = 0.4            # 띠 높이의 몇 % 이상 검으면 그 열을 "검은 열"로 봄
    edge_ratio: float = 0.05            # 화면 양끝 몇 % 안에 닿으면 "끝까지"
    corner_min_width: float = 2.0       # 코너 = 가로 폭이 라인 굵기의 이 배수 이상 (곧은 라인이 화면 끝에 있는 경우 제외)
    ahead_center_ratio: float = 0.4     # 먼 띠에서 가운데 ±40% 안의 세로 라인 = 앞쪽 라인
    default_line_px: int = 20           # 라인 굵기 추정이 안 될 때
    max_line_px: int = 45               # 한 줄에서 이보다 넓은 검은 구간은 라인이 아님 (두꺼운 물체, 붙은 물체)
    angle_min_rows: int = 15            # 각도를 믿으려면 라인을 따라간 줄이 이만큼은 있어야 함
    center_weight_min: float = 0.3      # 라인 고르기 가중치: 가운데 1.0 → 화면 양끝 이 값
    prev_weight_min: float = 0.5        # 직전 라인 위치에서 화면 폭 절반만큼 떨어지면 이 값
    bottom_bonus: float = 1.5           # 화면 맨 아래(차량 바로 앞)에 닿는 덩어리 가산
    confirm_frames: int = 3
    line_end_frames: int = 10           # 라인 끝만 더 오래 확인 — 흔들림·반사로 잠깐 놓친 것을 끝으로 보지 않게 (2026-10-30 S1)
    branch_top_ratio: float = 0.2       # 옆 갈래는 화면 위에서 이 비율 아래부터 끝까지 봄 (마커 앞에 섰을 때 분기점이 관심 영역보다 멂)
    branch_reach: float = 0.2           # 라인 중심에서 화면 폭의 이 비율 이상 옆으로 뻗어야 갈래
    branch_rows: int = 3                # 그런 줄이 이만큼 있어야 갈래
    sticker_as_line: bool = True        # 라인 위 색 스티커를 라인으로 (마커를 테이프 위에 붙임, 2026-11-06)
    sticker_s_min: int = 100            # 색 스티커 = 채도 이상·밝기 이상 (실측 스티커 S 140~250, 바닥 S 30 이하)
    sticker_v_min: int = 80


@dataclass
class LineResult:
    offset: float           # -1.0 ~ 1.0 (라인 중심 좌우 오차, 오른쪽 +)
    line_found: bool
    event: LineEvent        # 확정된 이벤트 (연속 프레임 통과분만)
    raw_event: LineEvent    # 이번 프레임 판정 (확인 전)
    angle: float = 90.0     # 라인 각도(도) — 90 = 똑바로, 90 미만 = 위쪽이 오른쪽으로 기움
    angle_valid: bool = False
    branch_left: bool = False   # 라인과 이어진 왼쪽 갈래가 보임 (마커 앞 회전 판단용)
    branch_right: bool = False


class EventConfirmer:
    """같은 이벤트가 frames번 쌓이면 그 이벤트, 아니면 NONE. per_event로 이벤트별 횟수를 따로 줄 수 있음.
    tolerant 이벤트는 NONE 한 프레임을 건너뛰어도 이어서 셈 — 멈춘 채 확인할 때 떨림으로 한 장 빠져도 처음부터 세지 않음."""

    def __init__(self, frames: int = 3, per_event: dict | None = None, tolerant: set | None = None):
        self.frames = frames
        self.per_event = per_event or {}
        self.tolerant = tolerant or set()
        self.last = LineEvent.NONE
        self.count = 0
        self.missed = False

    def push(self, event: LineEvent) -> LineEvent:
        if event == LineEvent.NONE and self.last in self.tolerant and not self.missed:
            self.missed = True                         # 한 프레임 빠짐 — 세던 것을 유지
            return LineEvent.NONE
        self.count = self.count + 1 if event == self.last else 1
        self.last = event
        self.missed = False
        need = self.per_event.get(event, self.frames)
        return event if event != LineEvent.NONE and self.count >= need else LineEvent.NONE


def black_mask(bgr: np.ndarray, cfg: LineConfig, spread: int = 0) -> np.ndarray:
    """바닥보다 충분히 어둡고 색이 없는 픽셀 = 255. 기준 = 줄마다 상위 10% 밝기(바닥) × black_ratio.
    고정 문턱이면 광택 테이프가 반사로 밝아질 때 라인을 잃음 (2026-11-02 실제 사진: 라인 V 약 124, 바닥 약 190)."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    s = hsv[..., 1]
    v = cv2.medianBlur(hsv[..., 2], cfg.black_blur)             # 테라조 바닥 무늬·테이프 반사 얼룩 완화
    floor = np.percentile(v, 90, axis=1, keepdims=True)
    if spread:      # 위아래 spread줄 중 가장 밝은 바닥 — 가로 테이프가 줄 전체를 덮으면 그 줄의 바닥이 검게 잡히는 것 방지
        floor = cv2.dilate(floor.astype(np.uint8), np.ones((2 * spread + 1, 1), np.uint8)).astype(float)
    mask = ((v <= cfg.black_ratio * floor) & (v <= cfg.black_v_max) & (s <= cfg.black_s_max)).astype(np.uint8) * 255
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def line_mask(bgr: np.ndarray, cfg: LineConfig, spread: int = 0) -> np.ndarray:
    """라인 마스크 = 검정 (+ sticker_as_line이면 채도 높은 색 스티커)."""
    mask = black_mask(bgr, cfg, spread)
    if cfg.sticker_as_line:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        color = ((hsv[..., 1] >= cfg.sticker_s_min) & (hsv[..., 2] >= cfg.sticker_v_min)).astype(np.uint8) * 255
        mask |= cv2.morphologyEx(color, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return mask


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
        self.confirm = EventConfirmer(self.cfg.confirm_frames, {LineEvent.LINE_END: self.cfg.line_end_frames},
                                      {LineEvent.CORNER_LEFT, LineEvent.CORNER_RIGHT})
        self.offset = 0.0
        self.line_px = float(self.cfg.default_line_px)     # 직전 라인 굵기 (정지선 폭 기준)
        self.lost_frames = 0                               # 가까운 띠에서 라인을 연속으로 놓친 프레임 수

    def set_frames(self, key: str, frames: int) -> None:
        """노트북 앱 조정 (DEV TUNE) — CONFIRM: 코너 확정 프레임, LINE_END: 라인 끝 확정 프레임."""
        frames = int(frames)
        if key == "CONFIRM":
            self.cfg.confirm_frames = self.confirm.frames = frames
        elif key == "LINE_END":
            self.cfg.line_end_frames = self.confirm.per_event[LineEvent.LINE_END] = frames
        else:
            raise ValueError(f"모르는 프레임 항목: {key}")

    def process(self, bgr: np.ndarray) -> LineResult:
        c = self.cfg
        img = cv2.resize(bgr, (c.width, c.height), interpolation=cv2.INTER_AREA)
        roi = img[int(c.height * (1 - c.roi_ratio)):, :]
        mask = line_mask(roi, c)
        h = mask.shape[0]
        _, labels = cv2.connectedComponents(mask, connectivity=8)
        line_id = self.select_line(labels, h)
        if line_id is None:                                   # 가까운 띠에 라인 없음 (주변 물체만 있거나 아무것도 없음)
            return self._lost(mask)
        line = np.where(labels == line_id, 255, 0).astype(np.uint8)   # 고른 라인 + 이어진 갈래만
        near = line[2 * h // 3:]                                # 오프셋은 가까운 띠, 이벤트는 detect_event가 전체에서
        centered = self.center_offset(near)
        if centered is None:                                  # 가까운 띠가 전부 두꺼움 (물체에 덮임)
            return self._lost(mask)
        self.offset, self.line_px = centered
        self.lost_frames = 0
        raw = self.detect_event(line, self.line_px)
        angle = self.line_angle(line)
        left, right = self.branch_sides(img)
        return LineResult(round(self.offset, 3), True, self.confirm.push(raw), raw,
                          round(angle, 1) if angle is not None else 90.0, angle is not None, left, right)

    def branch_sides(self, img: np.ndarray) -> tuple[bool, bool]:
        """(왼쪽 갈래, 오른쪽 갈래). 화면 branch_top_ratio 아래 전체에서 맨 아래 라인 위치와 이어진 덩어리를 골라
        아래→위로 라인 중심을 따라가며, 그 줄에서 라인을 포함한 검은 구간이 옆으로 branch_reach 이상 뻗었는지 센다.
        가는 줄에서만 중심을 옮김 — 분기점의 넓은 줄에서는 직전 중심 유지."""
        c = self.cfg
        region = img[int(c.height * c.branch_top_ratio):, :]
        mask = line_mask(region, c, spread=c.max_line_px)
        _, labels = cv2.connectedComponents(mask, connectivity=8)
        ref = (self.offset + 1) * c.width / 2
        bottom = labels[-max(3, mask.shape[0] // 10):, max(0, int(ref - c.max_line_px)): int(ref + c.max_line_px) + 1]
        ids, counts = np.unique(bottom[bottom > 0], return_counts=True)
        if not len(ids):
            return False, False
        comp = labels == ids[np.argmax(counts)]
        reach = c.branch_reach * c.width
        left = right = 0
        for y in range(comp.shape[0] - 1, -1, -1):
            spans = _spans(comp[y])
            if not spans:
                continue
            a, b = min(spans, key=lambda s: 0 if s[0] <= ref <= s[1] else min(abs(s[0] - ref), abs(s[1] - ref)))
            if not (a - c.max_line_px <= ref <= b + c.max_line_px):
                continue                                            # 라인과 떨어진 구간
            if b - a + 1 <= c.max_line_px:
                ref = (a + b) / 2
                continue
            left += ref - a >= reach
            right += b - ref >= reach
        return left >= c.branch_rows, right >= c.branch_rows

    def _lost(self, mask: np.ndarray) -> LineResult:
        """가까운 띠에서 라인을 못 찾음. 직전 라인 위치 근처 위쪽(중간·먼 띠)에 라인 모양이 보이면 잠깐 놓친 것(NONE),
        아니면 라인 끝(LINE_END, line_end_frames 연속이어야 확정). 오프셋은 마지막 값 유지.
        놓친 상태가 line_end_frames × 2를 넘으면 위쪽에 무엇이 보여도 라인 끝 — 멈추지 않고 계속 달리는 것을 막음."""
        self.lost_frames += 1
        keep_going = self.lost_frames < 2 * self.cfg.line_end_frames and self._line_above(mask)
        raw = LineEvent.NONE if keep_going else LineEvent.LINE_END
        return LineResult(round(self.offset, 3), False, self.confirm.push(raw), raw)

    def _line_above(self, mask: np.ndarray) -> bool:
        """중간·먼 띠의 직전 라인 위치 근처(±2 × max_line_px)에 가는 구간이 angle_min_rows/2 줄 이상 이어진 덩어리가 있는지."""
        c = self.cfg
        prev_x = (self.offset + 1) * c.width / 2
        reach = 2 * c.max_line_px
        upper = mask[: 2 * mask.shape[0] // 3]
        n, labels = cv2.connectedComponents(upper, connectivity=8)
        for i in range(1, n):
            comp = labels == i
            rows = 0
            for y in range(comp.shape[0]):
                runs = self._thin_runs(comp[y], prev_x)
                if runs and abs((runs[0][0] + runs[0][1]) / 2 - prev_x) <= reach:
                    rows += 1
            if rows >= max(3, c.angle_min_rows // 2):
                return True
        return False

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
        if left and right or (left or right) and ahead:        # T자·옆 갈래 = 갈림길 → 마커 인식
            return LineEvent.NONE
        if (left or right) and width < c.corner_min_width * line_px:           # 곧은 라인이 화면 끝에 있을 뿐 — 가로로 뻗지 않음
            return LineEvent.NONE
        if left:
            return LineEvent.CORNER_LEFT
        if right:
            return LineEvent.CORNER_RIGHT
        return LineEvent.NONE

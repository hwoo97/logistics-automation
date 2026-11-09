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
같은 판정이 confirm_frames번 쌓이면 확정(한 프레임 빠짐 허용).

2026-11-09 교수 점검(00_instructor/2026-11-09.md) 반영:
  ① 라인 굵기 = 정지선 높이의 굵기 — 차량에 가까운 끝(화면 아래, 유턴 뒤엔 위)에서 라인을 따라가며 줄마다 폭을 재고
     넓은 줄(정지선)에 닿으면 그 바로 앞 8줄의 중앙값. 관심 영역 전체 중앙값은 원근 때문에 가까운 굵은 쪽에 가까워
     먼 정지선의 두께가 기준과 거의 같아지고, 첫 판정 직후 판정이 사라져 재출발했다.
  ② 판정 직후(hold 중)는 두께 기준을 keep_thickness(0.5)로 낮춤 — 시작 기준보다 유지 기준을 낮게.
  ③ 마스크 = line_detect.line_mask (검정 + 라인 위 색 스티커) — 스티커 자리가 가로선의 빈 구간이 되지 않게.
  ④ 판정이 참이면 분기점인지 한 번 더 — 가로선이 양끝 너머로 junction_reach × 라인 굵기만큼 이어지거나(주),
     가로선 위에 스티커 색이 있으면(보조) 분기점으로 보고 제외. 비스듬히 다가간 T자 분기점에서 스티커 사이 구간이
     짧은 정지선으로 판정되던 것. 트랙 전제: 정지선 가로선 위에는 스티커를 붙이지 않는다.
"""
from dataclasses import dataclass

import cv2
import numpy as np

from agv_protocol.types import LineEvent
from agv_vision.line_detect import EventConfirmer, _spans, line_mask


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
    keep_thickness: float = 0.5         # 판정 직후(hold 중) 두께 기준 — 시작보다 유지 기준을 낮게 (2026-11-09 교수 점검 ②)
    side_min_ratio: float = 0.5         # 라인 바깥으로 양쪽 각각 라인 굵기 × 이 값 이상 이어져야 함
    gap_cols: int = 3                   # 정지선 열이 이만큼까지 끊겨도 이어진 것으로 봄 (테이프 반사)
    max_tilt_deg: float = 35.0          # 이보다 기울면 정지선 아님 (대각선 라인)
    edge_ratio: float = 0.05            # 화면 양끝 몇 % 안에 닿으면 "끝까지"
    confirm_frames: int = 3
    hold_frames: int = 20               # 판정 후 이 프레임 동안은 관심 영역 아래로 나가는 정지선도 인정 (지연·관성으로 지나쳐 멈춤)
    hold_max_thick: float = 2.5         # 그때 아래에 닿는 구간 두께 상한 (라인 굵기 배수) — 넘으면 라인으로 봄
    sticker_as_line: bool = True        # 이하 3개는 LineConfig와 같은 뜻 — 정지선 마스크에도 라인 위 스티커 포함 (③)
    sticker_s_min: int = 100
    sticker_v_min: int = 80
    junction_check: bool = True         # ④ 분기점 확인
    junction_reach: float = 3.0         # 가로선 양끝 너머로 라인 굵기의 이 배수만큼 이어지면 분기점
    junction_color_px: int = 20         # 가로선 위 스티커 색 화소가 이만큼 이상이면 분기점


@dataclass
class StopLineResult:
    detected: bool          # 이번 프레임 판정
    confirmed: bool         # confirm_frames 쌓여 확정
    angle: float = 0.0      # 정지선 기울기(도), 0 = 수평, + = 오른쪽이 위
    reason: str = ""        # 판정 안 됐을 때 이유 (진단 로그용)
    candidate: bool = False # 정지선 같은 두꺼운 가로 열이 라인 굵기 이상 보였는데 탈락 = 놓칠 뻔한 장면


def _longest_run(col: np.ndarray) -> tuple[int, int] | None:
    """세로 한 열에서 가장 긴 True 구간 (시작, 끝 포함)."""
    spans = _spans(col)
    return max(spans, key=lambda s: s[1] - s[0]) if spans else None


class StopLineDetector:
    def __init__(self, cfg: StopLineConfig | None = None):
        self.cfg = cfg or StopLineConfig()
        self.confirm = EventConfirmer(self.cfg.confirm_frames, tolerant={LineEvent.STOP_LINE})
        self.hold = 0                                 # 남은 "아래로 나가도 인정" 프레임
        self.reason, self.candidate = "", False       # 마지막 _find의 탈락 이유·후보 여부
        self.last_px = None                           # 마지막으로 잰 라인 굵기 (따라간 줄 전체)
        self.last_local = None                        # 마지막으로 잰 정지선 높이의 라인 굵기
        self.arm = None                               # 판정된 정지선 (위, 아래, 기울기, 절편, 왼, 오른, 라인 굵기)

    def set_frames(self, frames: int) -> None:
        """노트북 앱 "확인 프레임" (DEV TUNE CONFIRM)."""
        self.cfg.confirm_frames = self.confirm.frames = int(frames)

    def process(self, bgr: np.ndarray, offset: float) -> StopLineResult:
        """offset: line_node의 라인 오프셋 (-1~1)."""
        c = self.cfg
        img = cv2.resize(bgr, (c.width, c.height), interpolation=cv2.INTER_AREA)
        roi = img[int(c.height * (1 - c.roi_ratio)):, :]
        mask = line_mask(roi, c)                                                    # ③
        found, angle = self._find(mask, (offset + 1) * c.width / 2, self.hold > 0)
        if found and c.junction_check and self._is_junction(roi):                  # ④
            found, angle = False, 0.0
            self.reason = "분기점 (가로선이 양끝 너머로 이어지거나 가로선 위에 스티커)"
            self.candidate = True
        self.hold = c.hold_frames if found else max(0, self.hold - 1)
        event = LineEvent.STOP_LINE if found else LineEvent.NONE
        confirmed = self.confirm.push(event) == LineEvent.STOP_LINE
        return StopLineResult(found, confirmed, round(angle, 1),
                              "" if found else self.reason, (not found) and self.candidate)

    def _line_px(self, mask: np.ndarray, line_x: float) -> float:
        """정지선 높이의 라인 굵기 (①). 차량에 가까운 끝(화면 아래, 유턴 뒤에는 화면 위)에서 라인을 따라가며 줄마다
        폭을 재고 넓은 줄(정지선)에서 멈춘다. 정지선에 닿았으면 그 바로 앞 8줄의 중앙값, 아니면 따라간 줄 전체 중앙값.
        잴 수 없으면 직전 값, 그것도 없으면 default_line_px."""
        c = self.cfg
        best, best_hit = [], False
        cache = {}                                     # 줄마다 구간은 한 번만 계산 (아래·위 두 번 따라가도 시간 그대로)
        for rows in (range(mask.shape[0] - 1, -1, -1), range(mask.shape[0])):
            ref, widths, miss, hit = line_x, [], 0, False
            for y in rows:
                if y not in cache:
                    cache[y] = _spans(mask[y] > 0)
                near = [s for s in cache[y]
                        if s[0] - c.max_line_px / 2 <= ref <= s[1] + c.max_line_px / 2]
                if not near:
                    miss += 1
                    if widths and miss > 3:
                        break
                    continue
                a, b = min(near, key=lambda s: abs((s[0] + s[1]) / 2 - ref))
                if b - a + 1 > c.max_line_px:          # 정지선 줄에 닿음
                    if widths:
                        hit = True
                        break
                    continue
                widths.append(b - a + 1)
                ref, miss = (a + b) / 2, 0
            if len(widths) > len(best):
                best, best_hit = widths, hit
        if len(best) >= 5:
            self.last_px = float(np.median(best))
            self.last_local = float(np.median(best[-8:])) if best_hit else None
        elif len(best) >= 2 and best_hit:
            self.last_local = float(np.median(best))
        if self.last_local is not None:
            return self.last_local
        return self.last_px if self.last_px is not None else float(c.default_line_px)

    def _find(self, mask: np.ndarray, line_x: float, holding: bool = False) -> tuple[bool, float]:
        c = self.cfg
        h, w = mask.shape
        self.reason, self.candidate = "", False
        line_px = self._line_px(mask, line_x)
        # 라인과 이어진 덩어리 — 라인 위치 근처 픽셀이 가장 많은 것
        _, labels = cv2.connectedComponents(mask, connectivity=8)
        band = labels[:, max(0, int(line_x - line_px)): int(line_x + line_px) + 1]
        ids, counts = np.unique(band[band > 0], return_counts=True)
        if len(ids) == 0:
            self.reason = "라인 위치에 검정 없음"
            return False, 0.0
        comp = labels == ids[np.argmax(counts)]

        self.arm = None
        need = max(2.0, (c.keep_thickness if holding else c.min_thickness) * line_px)       # ②
        centers = np.full(w, np.nan)                  # 정지선 열의 세로 중심 (아니면 nan)
        runs = [None] * w                             # 정지선 열의 세로 구간
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
                runs[x] = run

        n_cols = int(np.count_nonzero(~np.isnan(centers)))
        self.candidate = n_cols >= line_px
        blocks = _spans(touch)
        if not blocks and not holding:
            self.reason = f"라인(위·아래 닿는 열) 없음, 정지선 열 {n_cols}"
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
            self.reason = (f"정지선 열이 {'왼쪽' if left is None else ''}{'·' if left is None and right is None else ''}"
                           f"{'오른쪽' if right is None else ''}에 없음 (라인 {line_l}~{line_r}, 굵기 {line_px:.0f}, 열 {n_cols})")
            return False, 0.0
        side = c.side_min_ratio * line_px
        if line_l - left < side or right - line_r < side:
            self.reason = f"한쪽이 짧음 (왼 {line_l - left}px, 오른 {right - line_r}px, 기준 {side:.0f}px)"
            return False, 0.0
        lo, hi = c.width_ratio
        if not lo * line_px <= right - left + 1 <= hi * line_px:
            self.reason = (f"폭 {right - left + 1}px = 라인 굵기 {line_px:.0f}px × {(right - left + 1) / line_px:.1f}"
                           f" (허용 {lo:g}~{hi:g})")
            return False, 0.0
        edge = max(1, int(w * c.edge_ratio))
        if left < edge and right >= w - edge:         # 양끝까지 = T자 → 마커 인식
            self.reason = "화면 양끝까지 (T자로 봄)"
            return False, 0.0
        xs = np.nonzero(~np.isnan(centers[left: right + 1]))[0] + left
        slope = np.polyfit(xs, centers[xs], 1)[0]     # 세로 중심 = slope·x + b (y는 아래로 증가)
        angle = float(np.degrees(np.arctan(-slope)))
        if abs(angle) > c.max_tilt_deg:
            self.reason = f"기울기 {angle:.0f}° (허용 ±{c.max_tilt_deg:g}°)"
            return False, angle
        # ④용 — 세로 범위·중심선은 라인에서 라인 굵기 1개 이상 떨어진 좌우 부분 열로만 (라인에 붙은 열은 근처 스티커까지 늘어남)
        arm = [int(x) for x in xs if (x <= line_l - line_px or x >= line_r + line_px) and runs[x][1] < h - 1]
        if len(arm) < 5:
            arm = [int(x) for x in xs]
        a_slope, a_b = np.polyfit(arm, centers[arm], 1)
        self.arm = (min(runs[x][0] for x in arm), max(runs[x][1] for x in arm), float(a_slope), float(a_b),
                    left, right, line_px)
        return True, angle

    def _is_junction(self, roi: np.ndarray) -> bool:
        """정지선 판정 뒤 추가 확인 (④) — 참이면 분기점으로 보고 정지선에서 제외."""
        c = self.cfg
        if self.arm is None:
            return False
        top, bottom, slope, b, left, right, lp = self.arm
        h, w = roi.shape[:2]
        # 색 확인(보조): 정지선 좌우 부분 위에 스티커 색
        if c.sticker_as_line:
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            color = ((hsv[..., 1] >= c.sticker_s_min) & (hsv[..., 2] >= c.sticker_v_min)).astype(np.uint8)
            color = cv2.morphologyEx(color, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            py, px = int(round(0.25 * lp)), int(round(lp))
            if color[max(0, top - py): bottom + py + 1, max(0, left - px): right + px + 1].sum() >= c.junction_color_px:
                return True
        # 이어짐 확인(주): 양끝 바깥으로 중심선을 따라 라인 굵기 junction_reach개 — 둘 다 이어지면 분기점
        m = line_mask(roi, c, c.max_line_px) > 0      # 스티커 포함 · 위아래 줄 바닥 기준 (가로선이 줄 전체를 덮어도 검정)
        half, span = max(2, round(0.5 * lp)), max(6, round(c.junction_reach * lp))
        edge = max(1, int(w * c.edge_ratio))

        def goes_on(x_end: int, step: int) -> bool:
            if (step < 0 and x_end < edge) or (step > 0 and x_end >= w - edge):
                return True                            # 이미 화면 가장자리
            dark = n = 0
            for k in range(1, span + 1):
                x = x_end + step * k
                if not 0 <= x < w:
                    break
                y = slope * x + b
                if y < 0 or y >= h:
                    return True                        # 중심선이 관심 영역 밖으로 벗어남
                # (y - half < 0으로 보면 관심 영역 맨 위로 막 들어온 정지선을 분기점으로 봄 — 2026-11-09 점검)
                dark += m[max(0, int(y) - half): int(y) + half + 1, x].mean() >= 0.5
                n += 1
            return n > 0 and dark / n >= 0.5

        return goes_on(left, -1) and goes_on(right, 1)

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


class MissTracker:
    """정지선 놓침 의심 구간 (진단 로그용, 2026-11-09) — 정지선 같은 두꺼운 가로 열(candidate)이 min_frames장 이상
    이어졌다가 사라질 때까지 판정이 한 번도 없으면, 그 구간을 한 번 알린다. 한두 장 스친 잡음은 무시.
    push()가 구간이 끝났을 때 (장 수, 이유 상위 3개, 마지막 이유)를 돌려주고, 아니면 None."""

    def __init__(self, min_frames: int = 3, gap: int = 1):
        self.min_frames, self.gap = min_frames, gap
        self._clear()

    def _clear(self):
        self.frames, self.missing, self.detected, self.last = 0, 0, False, ""
        self.reasons = {}

    def push(self, r: StopLineResult):
        if r.detected or r.candidate:
            self.frames += 1
            self.missing = 0
            self.detected |= r.detected
            if r.candidate:
                key = r.reason.split(" (")[0]
                self.reasons[key] = self.reasons.get(key, 0) + 1
                self.last = r.reason
            return None
        if self.frames == 0:
            return None
        self.missing += 1
        if self.missing <= self.gap:                    # 한 장 빠짐은 같은 구간
            return None
        out = None
        if not self.detected and self.frames >= self.min_frames:
            top = sorted(self.reasons.items(), key=lambda kv: -kv[1])[:3]
            out = (self.frames, top, self.last)
        self._clear()
        return out

"""카메라 팬 정면 자동 맞춤 (② 순수 로직, 2026-11-06).

차를 곧은 라인 위에 똑바로 놓고 시작한다. 카메라가 정면(차 진행 방향)을 보면 라인이 화면에서 수직(90°)으로 보인다
— 카메라가 차 중심에서 옆으로 비켜 달려 있어도 각도는 90°라서, 오프셋이 아니라 라인 각도로 맞춘다.
순서: 지금 팬에서 라인 각도 평균 → 팬을 step_deg 돌려 다시 평균 → 팬 1°당 각도 변화(기울기)를 구함
     → 기울기로 90°가 될 팬을 계산해 옮김 → 다시 재서 90° ± tolerance_deg 안이면 끝, 아니면 반복.
입력은 라인 상태(각도·유효 여부)와 시각, 출력은 Step(동작, 팬, 이유) — 서보·ROS는 pantilt_node가 맡는다.
"""
import json
import os
import time
from dataclasses import dataclass


@dataclass
class CalConfig:
    step_deg: float = 3.0           # 기울기를 재려고 처음 돌리는 각도
    tolerance_deg: float = 1.0      # 라인 각도가 90 ± 이 안이면 끝
    frames: int = 5                 # 한 위치에서 평균 낼 라인 각도 수
    settle_s: float = 0.5           # 팬을 돌린 뒤 이 시간은 재지 않음 (서보 흔들림·이전 영상)
    max_iter: int = 8               # 계산해서 옮기는 횟수
    max_move_deg: float = 10.0      # 한 번에 이보다 많이 돌리지 않음 (라인이 화면 밖으로 나가지 않게)
    min_gain: float = 0.2           # 팬 1°당 라인 각도가 이만큼은 변해야 함 (안 변하면 라인·카메라 문제)
    pan_min: float = 20.0
    pan_max: float = 120.0
    max_bad: int = 30               # 한 위치에서 라인·각도 없는 상태가 이만큼 오면 실패
    timeout_s: float = 30.0


@dataclass
class Step:
    action: str                     # MOVE(팬을 pan으로) / MEASURED(진행 보고) / DONE(정면 = pan) / FAIL(reason)
    pan: float = 0.0
    angle: float = 0.0              # MEASURED: 그 팬에서 잰 라인 각도
    reason: str = ""


class PanCalibrator:
    def __init__(self, cfg: CalConfig | None = None):
        self.cfg = cfg or CalConfig()
        self.active = False

    def start(self, pan: float, now: float) -> Step:
        """pan = 지금 팬 각도. 그 자리부터 잰다."""
        self.active = True
        self.started = now
        self.points: list[tuple[float, float]] = []      # (팬, 평균 라인 각도)
        self.gain = None
        self.iter = 0
        return self._move(pan, now)

    def cancel(self) -> None:
        self.active = False

    def _move(self, pan: float, now: float) -> Step:
        self.pan = round(min(self.cfg.pan_max, max(self.cfg.pan_min, pan)), 1)
        self.since, self.samples, self.bad = now, [], 0
        return Step("MOVE", self.pan)

    def _fail(self, reason: str) -> Step:
        self.active = False
        return Step("FAIL", self.pan, reason=reason)

    def tick(self, now: float) -> Step | None:
        if self.active and now - self.started > self.cfg.timeout_s:
            return self._fail(f"시간 초과 {self.cfg.timeout_s:.0f}초")
        return None

    def feed(self, angle: float, valid: bool, now: float) -> list[Step]:
        """라인 상태 한 장. valid = 라인을 찾았고 각도를 믿을 수 있음."""
        c = self.cfg
        if not self.active or now - self.since < c.settle_s:
            return []
        if not valid:
            self.bad += 1
            if self.bad >= c.max_bad:
                return [self._fail("라인이 안 보임 — 곧은 라인 위에 놓고 LINE 자세에서 라인이 보이는지 확인")]
            return []
        self.samples.append(angle)
        if len(self.samples) < c.frames:
            return []
        s = sorted(self.samples)
        measured = s[len(s) // 2]                         # 중앙값 — 한두 장 튀는 각도 무시
        self.points.append((self.pan, measured))
        steps = [Step("MEASURED", self.pan, measured)]
        return steps + [self._next(measured, now)]

    def _next(self, angle: float, now: float) -> Step:
        c = self.cfg
        err = angle - 90.0
        if abs(err) <= c.tolerance_deg:
            self.active = False
            return Step("DONE", self.pan, angle)
        if len(self.points) == 1:                         # 기울기를 재려고 한 번 돌림 — 90°에 가까워질 것 같은 쪽은 모름
            return self._move(self.pan + c.step_deg, now)
        (p0, a0), (p1, a1) = self.points[-2], self.points[-1]
        if abs(p1 - p0) >= 0.5:
            gain = (a1 - a0) / (p1 - p0)
            if abs(gain) >= c.min_gain and (self.gain is None or gain * self.gain > 0):
                self.gain = gain                          # 부호가 뒤집힌 값은 잡음으로 보고 버림
        if self.gain is None:
            return self._fail(f"팬을 돌려도 라인 각도가 거의 안 변함 ({a0:.1f}° → {a1:.1f}°)")
        self.iter += 1
        if self.iter > c.max_iter:
            return self._fail(f"{c.max_iter}번 안에 못 맞춤 (마지막 {angle:.1f}°)")
        move = max(-c.max_move_deg, min(c.max_move_deg, -err / self.gain))
        if abs(move) < 0.5:
            move = 0.5 if move > 0 else -0.5              # 서보 최소 단위보다 작으면 0.5°는 움직임
        return self._move(self.pan + move, now)


# ── 저장 파일 (~/.agv/pantilt_cal.json) — 팬 정면, SIGN 자세 틸트 (2026-11-06) ──

def load_cal(path: str) -> dict:
    """{"pan_front": 팬 정면 또는 None, "sign_tilt": SIGN 틸트 또는 None}. 파일이 없거나 깨졌으면 둘 다 None."""
    out = {"pan_front": None, "sign_tilt": None}
    try:
        with open(path, encoding="utf-8") as fp:
            data = json.load(fp)
    except (OSError, ValueError):
        return out
    if not isinstance(data, dict):
        return out
    for key in out:
        try:
            out[key] = float(data[key]) if data.get(key) is not None else None
        except (TypeError, ValueError):
            pass
    return out


def save_cal(path: str, pan_front: float | None, sign_tilt: float | None) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as fp:
        json.dump({"pan_front": pan_front, "sign_tilt": sign_tilt, "saved": time.strftime("%Y-%m-%d %H:%M:%S")},
                  fp, ensure_ascii=False, indent=2)

"""임무 상태 머신 (② 순수 로직) — 초기 설정(트랙 매핑)·작업(모양별 상차·하차) (functional_design 5절, 2026-11-02).

입력은 Event, 출력은 Action 목록. ROS·시간 함수에 의존하지 않는다 (시각은 이벤트의 now로 받음).

지점 도착 공통: NAVIGATE(정지선까지) → APPROACH(표지판 앞까지 서행, 초음파로 멈춤) → CHECK_SIGN(카메라 올려 분류)
  → 지점 역할 → 카메라 내리고 UTURN → NAVIGATE(유턴 뒤 첫 정지선 = 방금 지점의 정지선은 통과). 대기장소는 유턴 없이 끝.
초기 설정: 출발 위치(대기장소) 표지판 확인 → 매핑에 없는 색을 하나씩 목적지 색으로 따라가 표지판을 읽어 저장 → 4색 끝나면 대기장소로.
작업: 먼저 운반할 물품은 무작위, 한 물품을 다 옮긴 뒤 다음 물품. 1회 최대 9개(넘으면 여러 번 왕복), 상차 지점에서 1초에 1개
싣고 그 물품 하차 지점에서 1초에 1개 내림 → 끝나면 대기장소(Start)로. (교수 변경 시나리오 2026-11-03)
"""
import random
from dataclasses import dataclass, field

from agv_protocol.types import (
    CARGO_LED, CARGO_TARGET, COLORS, MAX_ITEMS, Cargo, DriveCommand, LedState, Mode, Pose, Sign, SIGN_TARGET, State, Target,
)

TARGETS = (Target.WORK, Target.DROP_A, Target.DROP_B, Target.PARK)


@dataclass
class MissionConfig:
    sign_distance: float = 0.18         # 표지판 앞 정지 거리(m) — 변경 시나리오 "약 18cm"
    sign_hits: int = 3                  # 이만큼 연속 가까우면 정지 (초음파 튐 방지)
    approach_timeout_s: float = 10.0    # 정지선에서 표지판 앞까지 시간 제한 → E_APPROACH
    item_interval_s: float = 1.0        # 상차·하차 1개 간격
    obstacle_enabled: bool = False      # 변경 시나리오: 장애물 없음 — 켜면 주행 중 가까운 물체에서 멈춤
    obstacle_distance: float = 0.15     # 주행 중 이보다 가까우면 장애물
    obstacle_clear_s: float = 1.0       # 장애물이 이만큼 연속 없으면 재개
    sign_retry: int = 1                 # 표지판 확정 실패 시 다시 보는 횟수
    parked_s: float = 2.0               # PARKED 표시 후 IDLE로


@dataclass
class Event:
    kind: str                           # SETUP START STOP RESUME MANUAL_ON MANUAL_OFF MOVE DRIVE_DONE PHASE RANGE POSE_READY SIGN TICK
    data: dict = field(default_factory=dict)


@dataclass
class Action:
    kind: str                           # DRIVE CANCEL_DRIVE TARGET POSE LED DIGIT CLASSIFY ESTOP CMD_VEL SAVE_MAP REPORT
    data: dict = field(default_factory=dict)


def A(kind: str, **data) -> Action:
    return Action(kind, data)


@dataclass
class MissionContext:
    mode: Mode = Mode.NONE
    plan: list = field(default_factory=list)        # 작업: [(Cargo, 개수 ≤ 9)] — 1회 적재 단위, 순서대로 실행
    done: int = 0                                   # 하차한 개수 (초기 설정: 확인한 지점 수)
    total: int = 0
    cargo: Cargo = Cargo.NONE
    load: int = 0
    target: Target = Target.WORK                    # 작업 목적지
    explore: str | None = None                      # 초기 설정: 지금 따라가는 색 (None = 출발 확인·복귀)
    track_map: dict = field(default_factory=dict)   # "WORK" → "blue"
    skip_stop_lines: int = 0                        # 다음 NAVIGATE에서 통과할 정지선
    retry: int = 0
    hits: int = 0
    since: float = 0.0                              # APPROACH 시작·PARKED 시작 시각
    next_tick: float = 0.0                          # 다음 상차·하차 시각
    clear_since: float | None = None                # 장애물 해제 시작 시각
    resume_state: State | None = None               # ESTOP·ERROR·PAUSED 뒤 돌아갈 상태
    error: str = ""


class MissionStateMachine:
    def __init__(self, cfg: MissionConfig | None = None, track_map: dict | None = None, rng=None):
        self.cfg = cfg or MissionConfig()
        self.rng = rng or random.Random()             # 먼저 운반할 물품 고르기 (시험에서는 고정 시드)
        self.state = State.IDLE
        self.ctx = MissionContext(track_map=dict(track_map or {}))
        self.now = 0.0
        self.phase = ""

    # ── 조회 ─────────────────────────────────────────────

    @property
    def mapped(self) -> bool:
        return all(self.ctx.track_map.get(t.value) for t in TARGETS)

    def status(self) -> dict:
        c = self.ctx
        if c.mode == Mode.SETUP:
            target = c.explore or c.track_map.get(Target.PARK.value) or "NONE"
        elif c.mode == Mode.WORK:
            target = c.target.value
        else:
            target = "NONE"
        return {"state": self.state.value, "mode": c.mode.value, "done": c.done, "total": c.total,
                "cargo": c.cargo.value, "load": c.load, "target": target, "map": dict(c.track_map),
                "message": c.error}

    # ── 입력 ─────────────────────────────────────────────

    def handle(self, ev: Event) -> list[Action]:
        self.now = ev.data.get("now", self.now)
        k = ev.kind
        if k == "STOP":
            return self._estop()
        if k == "RESUME":
            return self._resume() if self.state in (State.ESTOP, State.ERROR) else []
        if k == "MANUAL_ON" and self.state in (State.IDLE, State.ESTOP, State.ERROR):
            self._reset()
            self.state = State.MANUAL
            return [A("CANCEL_DRIVE"), A("LED", state=LedState.OFF.value), A("DIGIT", char="0")]
        if k == "MANUAL_OFF" and self.state == State.MANUAL:
            self.state = State.IDLE
            return [A("CMD_VEL", linear=0.0, angular=0.0)]
        if k == "MOVE":
            return [A("CMD_VEL", **ev.data)] if self.state == State.MANUAL else []
        if k == "SETUP" and self.state == State.IDLE:
            return self._setup()
        if k == "START" and self.state == State.IDLE:
            return self._start(int(ev.data.get("circle", 0)), int(ev.data.get("square", 0)))
        if k == "PHASE":
            return self._on_phase(ev.data.get("phase", ""))
        handler = getattr(self, f"_on_{k.lower()}", None)
        return handler(ev.data) if handler else []

    # ── 시작 ─────────────────────────────────────────────

    def _reset(self) -> None:
        c = self.ctx
        c.mode, c.plan, c.done, c.total, c.cargo, c.load = Mode.NONE, [], 0, 0, Cargo.NONE, 0
        c.explore, c.skip_stop_lines, c.resume_state, c.error = None, 0, None, ""

    def _setup(self) -> list[Action]:
        """출발 위치(대기장소 표지판 앞)를 먼저 확인한 뒤 색 탐색. 이전 매핑은 지운다 (트랙이 바뀌었을 수 있음)."""
        self._reset()
        c = self.ctx
        c.mode, c.total, c.track_map = Mode.SETUP, len(TARGETS), {}
        acts = [A("SAVE_MAP", track_map={}), A("DIGIT", char="0"), A("LED", state=LedState.OFF.value),
                A("REPORT", event="SETUP_STARTED")]
        return acts + self._check_sign()

    def _start(self, circle: int, square: int) -> list[Action]:
        if not self.mapped:
            return [A("REPORT", error="E_MAP", detail="초기 설정 필요")]
        plan = self.make_plan(circle, square)
        if not plan:
            return []
        self._reset()
        c = self.ctx
        c.mode, c.plan, c.total = Mode.WORK, plan, circle + square
        acts = [A("DIGIT", char="0"), A("LED", state=LedState.OFF.value),
                A("REPORT", event="STARTED", detail=f"적 {circle} 녹 {square}")]
        return acts + self._trip(Target.WORK)

    def make_plan(self, circle: int, square: int) -> list:
        """물품별 수량 → 1회 적재 목록. 먼저 운반할 물품은 무작위, 한 물품을 다 옮긴 뒤 다음, 1회 최대 9개."""
        order = [(cargo, n) for cargo, n in ((Cargo.CIRCLE, circle), (Cargo.SQUARE, square)) if n > 0]
        self.rng.shuffle(order)
        plan = []
        for cargo, n in order:
            while n > 0:
                plan.append((cargo, min(MAX_ITEMS, n)))
                n -= plan[-1][1]
        return plan

    # ── 이동 ─────────────────────────────────────────────

    def _trip(self, target: Target | None = None, color: str | None = None) -> list[Action]:
        """카메라를 내리고 유턴 → 목적지 색을 따라감 (유턴 뒤 첫 정지선 통과)."""
        if target is not None:
            self.ctx.target = target
            color = self.ctx.track_map[target.value]
        self.ctx.skip_stop_lines = 1
        self.state = State.UTURN
        return [A("POSE", pose=Pose.LINE.value), A("TARGET", color=color),
                A("DRIVE", command=DriveCommand.UTURN.value, skip=0)]

    def _navigate(self) -> list[Action]:
        self.state = State.NAVIGATE
        return [A("DRIVE", command=DriveCommand.FOLLOW_TO_EVENT.value, skip=self.ctx.skip_stop_lines)]

    def _approach(self) -> list[Action]:
        self.state = State.APPROACH
        self.ctx.hits, self.ctx.since = 0, self.now
        return [A("DRIVE", command=DriveCommand.APPROACH.value, skip=0)]

    def _check_sign(self) -> list[Action]:
        self.state = State.CHECK_SIGN
        self.ctx.retry = 0
        return [A("POSE", pose=Pose.SIGN.value)]

    def _on_phase(self, phase: str) -> list[Action]:
        """drive_node 피드백 — 정지선 통과가 끝나면 다시 재개할 때 통과하지 않게."""
        if self.phase == "PASS STOP_LINE" and phase != self.phase and self.state == State.NAVIGATE:
            self.ctx.skip_stop_lines = 0
        self.phase = phase
        return []

    def _on_drive_done(self, d: dict) -> list[Action]:
        result = d.get("result", "")
        if result in ("STOPPED", "E_ESTOP"):                # 우리가 취소했거나 비상 정지 — 각 경로에서 처리
            return []
        if self.state == State.UTURN:
            if result == "UTURN_DONE":
                return self._navigate()
            return self._error(result or "E_TURN", "유턴 실패")
        if self.state == State.NAVIGATE:
            if result == "STOP_LINE":
                self.ctx.skip_stop_lines = 0
                return self._approach()
            if result == "LINE_END":
                return self._error("E_LINE", "정지선 없이 라인 끝")
            return self._error("E_LINE" if not result.startswith("E_") else result, f"주행 중단 {result}")
        if self.state == State.APPROACH:
            return self._error("E_APPROACH", f"접근 중 주행 중단 {result}")
        return []

    # ── 센서 ─────────────────────────────────────────────

    def _on_range(self, d: dict) -> list[Action]:
        dist, c, cfg = d.get("distance"), self.ctx, self.cfg
        if dist is None:
            return []
        if self.state == State.APPROACH:
            c.hits = c.hits + 1 if dist <= cfg.sign_distance else 0
            if c.hits >= cfg.sign_hits:
                return [A("CANCEL_DRIVE")] + self._check_sign()
            return []
        if cfg.obstacle_enabled and self.state in (State.NAVIGATE, State.UTURN) and dist <= cfg.obstacle_distance:
            c.resume_state, c.clear_since = self.state, None
            self.state = State.PAUSED_OBSTACLE
            return [A("CANCEL_DRIVE"), A("REPORT", event="OBSTACLE", detail=f"{dist:.2f}m")]
        if self.state == State.PAUSED_OBSTACLE:
            if dist <= cfg.obstacle_distance:
                c.clear_since = None
            elif c.clear_since is None:
                c.clear_since = self.now
            elif self.now - c.clear_since >= cfg.obstacle_clear_s:
                return [A("REPORT", event="RESUMED", detail="장애물 없음")] + self._reenter(c.resume_state)
        return []

    def _on_pose_ready(self, d: dict) -> list[Action]:
        if self.state == State.CHECK_SIGN and d.get("pose") == Pose.SIGN.value:
            return [A("CLASSIFY", mode="SIGN")]
        return []

    def _on_sign(self, d: dict) -> list[Action]:
        if self.state != State.CHECK_SIGN:
            return []
        c = self.ctx
        label = d.get("label", "")
        if not label:
            if c.retry < self.cfg.sign_retry:
                c.retry += 1
                return [A("CLASSIFY", mode="SIGN")]
            return self._error("E_MAP" if c.mode == Mode.SETUP else "E_SIGN", f"표지판 확정 못 함 {d.get('message', '')}".strip())
        try:
            seen = SIGN_TARGET[Sign(label)]
        except ValueError:
            return self._error("E_SIGN", f"모르는 라벨 {label}")
        return self._arrive_setup(seen) if c.mode == Mode.SETUP else self._arrive_work(seen)

    def _arrive_setup(self, seen: Target) -> list[Action]:
        c = self.ctx
        acts = []
        if c.explore is None and not c.track_map:          # 출발 위치 확인
            if seen != Target.PARK:
                return self._error("E_MAP", f"출발 위치가 대기장소가 아님 ({seen.value})")
        elif c.explore is not None:
            known = c.track_map.get(seen.value)
            if known and known != c.explore:
                return self._error("E_MAP", f"{seen.value}가 {known}·{c.explore} 두 색에서 나옴")
            c.track_map[seen.value] = c.explore
            c.done = len(c.track_map)
            acts += [A("SAVE_MAP", track_map=dict(c.track_map)),
                     A("REPORT", event="MAPPED", detail=f"{seen.value} {c.explore}")]
        elif seen != Target.PARK:                           # 4색 끝, 대기장소로 복귀 중
            return self._error("E_MAP", f"대기장소 대신 {seen.value}")
        left = [color for color in COLORS if color not in c.track_map.values()]
        if left:
            c.explore = left[0]
            return acts + self._trip(color=c.explore)
        if seen == Target.PARK:
            return acts + self._finish("MAP_DONE")
        c.explore = None
        return acts + self._trip(color=c.track_map[Target.PARK.value])

    def _arrive_work(self, seen: Target) -> list[Action]:
        c = self.ctx
        if seen != c.target:
            return self._error("E_SIGN", f"{c.target.value} 기대, {seen.value} 확인")
        if seen == Target.PARK:
            return self._finish("PARKED")
        c.next_tick = self.now + self.cfg.item_interval_s
        if seen == Target.WORK:
            c.cargo, c.load = c.plan[0][0], 0
            self.state = State.LOAD
        else:
            self.state = State.UNLOAD
        return []

    def _finish(self, event: str) -> list[Action]:
        c = self.ctx
        self.state, c.since = State.PARKED, self.now
        return [A("POSE", pose=Pose.LINE.value), A("TARGET", color="NONE"), A("DIGIT", char="0"),
                A("LED", state=LedState.OFF.value), A("REPORT", event=event)]

    # ── 시간 ─────────────────────────────────────────────

    def _on_tick(self, d: dict) -> list[Action]:
        c, cfg = self.ctx, self.cfg
        if self.state == State.APPROACH and self.now - c.since > cfg.approach_timeout_s:
            return [A("CANCEL_DRIVE")] + self._error("E_APPROACH", "표지판 앞에 도달 못 함")
        if self.state == State.PARKED and self.now - c.since >= cfg.parked_s:
            self.state = State.IDLE
            c.mode = Mode.NONE
            return []
        if self.state not in (State.LOAD, State.UNLOAD) or self.now < c.next_tick:
            return []
        c.next_tick += cfg.item_interval_s
        if self.state == State.LOAD:
            c.load += 1
            acts = [A("DIGIT", char=str(c.load))]
            if c.load == 1:
                acts.append(A("LED", state=CARGO_LED[c.cargo].value))
            if c.load >= c.plan[0][1]:
                acts.append(A("REPORT", event="LOADED", detail=f"{c.cargo.value} {c.load}"))
                acts += self._trip(CARGO_TARGET[c.cargo])
            return acts
        c.load -= 1
        c.done += 1
        acts = [A("DIGIT", char=str(c.load))]
        if c.load == 0:
            cargo, n = c.plan.pop(0)
            c.cargo = Cargo.NONE
            acts += [A("LED", state=LedState.OFF.value), A("REPORT", event="UNLOADED", detail=f"{cargo.value} {n}")]
            acts += self._trip(Target.WORK if c.plan else Target.PARK)
        return acts

    # ── 비상 정지·오류·재개 ─────────────────────────────────

    def _estop(self) -> list[Action]:
        """비상 정지 — 장애물 정지 중이면 장애물 전 동작, 오류 중이면 오류 전 동작을 재개 대상으로 유지."""
        if self.state == State.MANUAL:
            self.ctx.resume_state = State.IDLE
        elif self.state not in (State.ESTOP, State.ERROR, State.PAUSED_OBSTACLE):
            self.ctx.resume_state = self.state
        self.state = State.ESTOP
        return [A("ESTOP", on=True), A("CANCEL_DRIVE")]

    def _error(self, code: str, detail: str) -> list[Action]:
        c = self.ctx
        c.resume_state, c.error = self.state, f"{code} {detail}"
        self.state = State.ERROR
        return [A("CANCEL_DRIVE"), A("LED", state=LedState.ERROR.value), A("DIGIT", char="E"),
                A("REPORT", error=code, detail=detail)]

    def _resume(self) -> list[Action]:
        c = self.ctx
        was_estop = self.state == State.ESTOP
        target, c.error = c.resume_state or State.IDLE, ""
        acts = [A("ESTOP", on=False)] if was_estop else []
        acts += [A("LED", state=CARGO_LED[c.cargo].value if c.cargo != Cargo.NONE else LedState.OFF.value),
                 A("DIGIT", char=str(c.load))]
        return acts + [A("REPORT", event="RESUMED")] + self._reenter(target)

    def _reenter(self, state: State | None) -> list[Action]:
        """멈춘 동작을 다시 시작한다 (비상 정지·오류·장애물 뒤)."""
        c = self.ctx
        if state == State.UTURN:
            self.state = State.UTURN
            return [A("POSE", pose=Pose.LINE.value), A("DRIVE", command=DriveCommand.UTURN.value, skip=0)]
        if state == State.NAVIGATE:
            return [A("POSE", pose=Pose.LINE.value)] + self._navigate()
        if state == State.APPROACH:
            return [A("POSE", pose=Pose.LINE.value)] + self._approach()
        if state == State.CHECK_SIGN:
            return self._check_sign()
        if state in (State.LOAD, State.UNLOAD):
            self.state, c.next_tick = state, self.now + self.cfg.item_interval_s
            return []
        if state == State.PARKED:
            return self._finish("MAP_DONE" if c.mode == Mode.SETUP else "PARKED")
        self.state = State.IDLE
        self._reset()
        return [A("TARGET", color="NONE")]

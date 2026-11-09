"""임무 상태 머신 (② 순수 로직) — 초기 설정(트랙 매핑)·작업(모양별 상차·하차) (functional_design 5절, 2026-11-02).

입력은 Event, 출력은 Action 목록. ROS·시간 함수에 의존하지 않는다 (시각은 이벤트의 now로 받음).

지점 도착 공통 (교수 변경 시나리오 2026-11-03 순서): NAVIGATE(정지선까지) → 정지선에서 카메라를 표지판 쪽으로 →
  APPROACH(직진 서행, 초음파 약 18cm에서 멈춤) → CHECK_SIGN(분류) → 지점 역할 → 카메라 내리고 UTURN
  → NAVIGATE(유턴 뒤 첫 정지선 = 방금 지점의 정지선은 통과). 대기장소는 유턴 없이 끝.
도착 시험 (TEST_ARRIVE, 노트북 앱 개발자 탭): 매핑 없이 지금 목적지 색으로 NAVIGATE → 위 도착 과정 → 분류 결과를
  EVENT ARRIVED로 알리고 카메라를 내린 뒤 IDLE (모델이 없어도 오류로 멈추지 않음). CANCEL = 진행 중 동작을 멈추고 IDLE.
초기 설정 (2026-11-06): 출발 표지판은 읽지 않고 유턴 → 빨강·파랑·초록·노랑 순서로 따라가 표지판을 읽어 저장 →
  세 곳을 읽으면 남은 한 곳(지점·색)은 소거법으로 저장 → 대기장소로 돌아가 표지판 확인 후 끝.
작업: 먼저 운반할 물품은 무작위, 한 물품을 다 옮긴 뒤 다음 물품. 1회 최대 9개(넘으면 여러 번 왕복), 상차 지점에서 1초에 1개
싣고 그 물품 하차 지점에서 1초에 1개 내림 → 끝나면 대기장소(Start)로. (교수 변경 시나리오 2026-11-03)
"""
import random
from dataclasses import dataclass, field

from agv_protocol.types import (
    CARGO_LED, CARGO_TARGET, COLORS, MAX_ITEMS, Cargo, DriveCommand, LedState, Mode, Pose, Sign, SIGN_TARGET, State, Target,
)

TARGETS = (Target.WORK, Target.DROP_A, Target.DROP_B, Target.PARK)
SETUP_ORDER = ("red", "blue", "green", "yellow")   # 초기 설정 탐색 순서 (2026-11-06 사용자 결정)


@dataclass
class MissionConfig:
    sign_distance: float = 0.30         # 표지판 앞 정지 거리(m) — 2026-11-06 사용자 결정 30cm (변경 시나리오는 "약 18cm")
    sign_settle_s: float = 1.0          # 멈추고 카메라 자세가 잡힌 뒤 이 시간 기다렸다 분류 (차체 흔들림 가라앉힘)
    start_sign_distance: float = 0.5    # 출발(SETUP·START) 때 초음파가 이보다 가까우면 대기장소 표지판을 보고 있음 → 유턴부터
                                        # (2026-11-06: 처음 차 방향을 모름). 멀거나 값이 없으면 유턴 없이 바로 라인을 따라감
    sign_hits: int = 3                  # 이만큼 연속 가까우면 정지 (초음파 튐 방지)
    approach_timeout_s: float = 10.0    # 정지선에서 표지판 앞까지 시간 제한 → E_APPROACH
    item_interval_s: float = 1.0        # 상차·하차 1개 간격
    obstacle_enabled: bool = False      # 변경 시나리오: 장애물 없음 — 켜면 주행 중 가까운 물체에서 멈춤
    obstacle_distance: float = 0.18     # 주행 중 이보다 가까우면 장애물 (drive_node 초음파 정지 뒤 재개 판단에도 씀)
    obstacle_clear_s: float = 1.0       # 장애물이 이만큼 연속 없으면 재개
    sign_retry: int = 1                 # 표지판 확정 실패 시 다시 보는 횟수
    parked_s: float = 2.0               # PARKED 표시 후 IDLE로


@dataclass
class Event:
    kind: str                           # SETUP START TEST_ARRIVE CANCEL STOP RESUME MANUAL_ON MANUAL_OFF MOVE DRIVE_DONE PHASE RANGE POSE_READY SIGN TICK
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
    explore: str | None = None                      # 초기 설정: 지금 따라가는 색 (None = 대기장소로 복귀)
    track_map: dict = field(default_factory=dict)   # "WORK" → "blue"
    skip_stop_lines: int = 0                        # 다음 NAVIGATE에서 통과할 정지선
    retry: int = 0
    hits: int = 0
    aiming: bool = False                            # 정지선에서 카메라를 표지판 쪽으로 올리는 중 (APPROACH 주행 전)
    test: bool = False                              # 도착 시험 중
    since: float = 0.0                              # APPROACH 시작·PARKED 시작 시각
    next_tick: float = 0.0                          # 다음 상차·하차 시각
    classify_at: float | None = None                # CHECK_SIGN: 이 시각에 분류 시작 (멈춘 뒤 sign_settle_s)
    front_range: float | None = None                # 마지막 초음파 거리 (값 없음 = None) — 출발 방향 판단
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
            release = self.state in (State.ESTOP, State.ERROR)   # 사람이 직접 조종 → 비상 정지 해제 (교수 점검 2.1)
            self._reset()
            self.state = State.MANUAL
            acts = [A("CANCEL_DRIVE"), A("LED", state=LedState.OFF.value), A("DIGIT", char="0")]
            return ([A("ESTOP", on=False)] if release else []) + acts
        if k == "MANUAL_OFF" and self.state == State.MANUAL:
            self.state = State.IDLE
            return [A("CMD_VEL", linear=0.0, angular=0.0)]
        if k == "MOVE":
            return [A("CMD_VEL", **ev.data)] if self.state == State.MANUAL else []
        if k == "SETUP" and self.state == State.IDLE:
            return self._setup()
        if k == "TEST_ARRIVE" and self.state == State.IDLE:
            return self._test_arrive()
        if k == "CANCEL":
            return self._cancel()
        if k == "RESET":
            return self._reset_all()
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
        # front_range는 지우지 않음 — 센서 값이라 모드와 상관없이 마지막 값을 씀
        c.aiming = c.test = False

    def _setup(self) -> list[Action]:
        """초기 설정 (2026-11-06 변경): 출발 표지판은 확인하지 않고 바로 유턴 → 빨강·파랑·초록·노랑 순서로 따라가
        표지판을 읽어 저장. 세 곳을 읽으면 남은 한 곳은 소거법으로 저장하고 대기장소로 돌아가 확인.
        이전 매핑은 지운다 (트랙이 바뀌었을 수 있음)."""
        self._reset()
        c = self.ctx
        c.mode, c.total, c.track_map = Mode.SETUP, len(TARGETS), {}
        c.explore = SETUP_ORDER[0]
        acts = [A("SAVE_MAP", track_map={}), A("DIGIT", char="0"), A("LED", state=LedState.OFF.value),
                A("REPORT", event="SETUP_STARTED")]
        return acts + self._trip(color=c.explore, depart=True)

    def _test_arrive(self) -> list[Action]:
        """도착 시험 — 목적지 색은 drive_node에 이미 정한 것(DEV TARGET)을 그대로 씀."""
        self._reset()
        self.ctx.test = True
        return [A("POSE", pose=Pose.LINE.value), A("REPORT", event="ARRIVE_TEST")] + self._navigate()

    def _cancel(self) -> list[Action]:
        """진행 중 동작을 멈추고 IDLE (개발자 탭 주행 정지). 비상 정지·수동은 그대로 둠."""
        if self.state in (State.IDLE, State.MANUAL, State.ESTOP):
            return []
        self.state = State.IDLE
        self._reset()
        return [A("CANCEL_DRIVE"), A("POSE", pose=Pose.LINE.value), A("LED", state=LedState.OFF.value),
                A("DIGIT", char="0"), A("REPORT", event="CANCELLED")]

    def _reset_all(self) -> list[Action]:
        """시험 초기화 (개발자 탭 "초기화", DEV RESET — 2026-11-09). 어느 상태에서든 주행을 멈추고 비상 정지를 풀고
        카메라를 내린 뒤 IDLE. 매핑이 4곳 다 안 됐으면(초기 설정 중간) 지운다 — 다 된 매핑은 남겨 작업 시험에 씀."""
        c = self.ctx
        self._reset()
        self.state = State.IDLE
        acts = [A("CANCEL_DRIVE"), A("ESTOP", on=False), A("CMD_VEL", linear=0.0, angular=0.0),
                A("POSE", pose=Pose.LINE.value), A("TARGET", color="NONE"),
                A("LED", state=LedState.OFF.value), A("DIGIT", char="0")]
        cleared = bool(c.track_map) and not self.mapped
        if cleared:
            c.track_map = {}
            acts.append(A("SAVE_MAP", track_map={}))
        return acts + [A("REPORT", event="RESET", detail="매핑 지움" if cleared else "")]

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
        return acts + self._trip(Target.WORK, depart=True)

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

    def _trip(self, target: Target | None = None, color: str | None = None, depart: bool = False) -> list[Action]:
        """카메라를 내리고 유턴 → 목적지 색을 따라감 (유턴 뒤 첫 정지선 통과).
        depart(SETUP·START 첫 출발): 차 방향을 모르므로 초음파로 판단 — 앞이 start_sign_distance보다 가까우면
        대기장소 표지판을 보고 있는 것 → 유턴, 아니면 이미 바깥을 보고 있음 → 유턴 없이 바로 따라감.
        어느 쪽이든 대기장소 정지선은 지나감."""
        c = self.ctx
        if target is not None:
            c.target = target
            color = c.track_map[target.value]
        c.skip_stop_lines = 1
        acts = [A("POSE", pose=Pose.LINE.value), A("TARGET", color=color)]
        if depart:
            facing = c.front_range is not None and c.front_range <= self.cfg.start_sign_distance
            seen = f"앞 {c.front_range:.2f}m" if c.front_range is not None else "앞 거리 없음"
            acts.append(A("REPORT", event="DEPART", detail=f"{'UTURN' if facing else 'STRAIGHT'} {seen}"))
            if not facing:
                return acts + self._navigate()
        self.state = State.UTURN
        return acts + [A("DRIVE", command=DriveCommand.UTURN.value, skip=0)]

    def _navigate(self) -> list[Action]:
        self.state = State.NAVIGATE
        return [A("DRIVE", command=DriveCommand.FOLLOW_TO_EVENT.value, skip=self.ctx.skip_stop_lines)]

    def _aim(self) -> list[Action]:
        """정지선에 섬 → 카메라를 표지판 쪽으로. 자세가 잡히면(POSE_READY) 표지판까지 직진 서행."""
        self.state = State.APPROACH
        self.ctx.aiming, self.ctx.hits, self.ctx.since = True, 0, self.now
        return [A("POSE", pose=Pose.SIGN.value)]

    def _approach(self) -> list[Action]:
        self.state = State.APPROACH
        self.ctx.aiming, self.ctx.hits, self.ctx.since = False, 0, self.now
        return [A("DRIVE", command=DriveCommand.APPROACH.value, skip=0)]

    def _check_sign(self) -> list[Action]:
        self.state = State.CHECK_SIGN
        self.ctx.retry, self.ctx.classify_at = 0, None
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
                return self._aim()
            if result == "LINE_END":
                return self._error("E_LINE", "정지선 없이 라인 끝")
            if result == "OBSTACLE":                        # drive_node 초음파 정지 → 물체가 치워지면 재개
                c = self.ctx
                c.resume_state, c.clear_since = State.NAVIGATE, None
                self.state = State.PAUSED_OBSTACLE
                return [A("REPORT", event="OBSTACLE", detail="초음파 정지")]
            return self._error("E_LINE" if not result.startswith("E_") else result, f"주행 중단 {result}")
        if self.state == State.APPROACH:
            return self._error("E_APPROACH", f"접근 중 주행 중단 {result}")
        return []

    # ── 센서 ─────────────────────────────────────────────

    def _on_range(self, d: dict) -> list[Action]:
        dist, c, cfg = d.get("distance"), self.ctx, self.cfg
        c.front_range = dist                                # 출발 방향 판단용 (값 없음도 그대로)
        if dist is None:
            if self.state != State.PAUSED_OBSTACLE:
                return []
            dist = float("inf")                             # 장애물 대기 중 값 없음 = 2m 안에 물체 없음 (교수 점검 2.6)
        if self.state == State.APPROACH and c.aiming:
            return []                                       # 아직 정지선에서 카메라를 올리는 중
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
        if self.state == State.APPROACH and self.ctx.aiming and d.get("pose") == Pose.SIGN.value:
            return self._approach()
        if self.state == State.CHECK_SIGN and d.get("pose") == Pose.SIGN.value:
            if self.cfg.sign_settle_s > 0:
                self.ctx.classify_at = self.now + self.cfg.sign_settle_s     # TICK에서 시작
                return []
            return [A("CLASSIFY", mode="SIGN")]
        return []

    def _on_sign(self, d: dict) -> list[Action]:
        """표지판 분류 결과 — 매번 EVENT SIGN <라벨> <점수> (못 읽으면 SIGN - <이유>)를 앱에 알린 뒤 처리."""
        if self.state != State.CHECK_SIGN:
            return []
        label = d.get("label", "")
        detail = f"{label} {float(d.get('score', 0.0)):.2f}" if label else f"- {d.get('message', '')}".rstrip()
        return [A("REPORT", event="SIGN", detail=detail)] + self._sign_result(d)

    def _sign_result(self, d: dict) -> list[Action]:
        c = self.ctx
        label = d.get("label", "")
        if c.test:
            return self._test_result(label, d.get("message", ""))
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

    def _test_result(self, label: str, message: str) -> list[Action]:
        """도착 시험 끝 — 분류 결과만 알리고 카메라를 내려 IDLE (유턴은 개발자 탭 U턴 버튼)."""
        c = self.ctx
        if not label and c.retry < self.cfg.sign_retry:
            c.retry += 1
            return [A("CLASSIFY", mode="SIGN")]
        self.state = State.IDLE
        self._reset()
        detail = label or f"표지판 못 읽음 {message}".strip()
        return [A("POSE", pose=Pose.LINE.value), A("REPORT", event="ARRIVED", detail=detail)]

    def _arrive_setup(self, seen: Target) -> list[Action]:
        c = self.ctx
        acts = []
        if c.explore is not None:
            known = c.track_map.get(seen.value)
            if known and known != c.explore:
                return self._error("E_MAP", f"{seen.value}가 {known}·{c.explore} 두 색에서 나옴")
            c.track_map[seen.value] = c.explore
            acts += [A("REPORT", event="MAPPED", detail=f"{seen.value} {c.explore}")]
            if len(c.track_map) == len(TARGETS) - 1:        # 세 곳 읽음 → 남은 한 곳은 소거법
                target = next(t.value for t in TARGETS if t.value not in c.track_map)
                color = next(x for x in SETUP_ORDER if x not in c.track_map.values())
                c.track_map[target] = color
                acts += [A("REPORT", event="MAPPED", detail=f"{target} {color} 소거")]
            c.done = len(c.track_map)
            acts.insert(0, A("SAVE_MAP", track_map=dict(c.track_map)))
        elif seen != Target.PARK:                           # 매핑 끝, 대기장소로 복귀 중
            return self._error("E_MAP", f"대기장소 대신 {seen.value}")
        left = [color for color in SETUP_ORDER if color not in c.track_map.values()]
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
        if self.state == State.CHECK_SIGN and c.classify_at is not None and self.now >= c.classify_at:
            c.classify_at = None
            return [A("CLASSIFY", mode="SIGN")]
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
        target, c.error = c.resume_state or State.IDLE, ""
        acts = [A("ESTOP", on=False)]                       # ERROR에서도 해제 — 비상 정지가 남아 있을 수 있음 (교수 점검 2.1)
        acts += [A("LED", state=CARGO_LED[c.cargo].value if c.cargo != Cargo.NONE else LedState.OFF.value),
                 A("DIGIT", char=str(c.load))]
        return acts + [A("REPORT", event="RESUMED")] + self._reenter(target)

    def _route_color(self) -> str | None:
        """지금 따라가야 할 목적지 색 (초기 설정: 탐색 색 또는 대기장소 색, 작업: 목적지 지점 색). 도착 시험은 None(앱 값)."""
        c = self.ctx
        if c.mode == Mode.SETUP:
            return c.explore or c.track_map.get(Target.PARK.value)
        if c.mode == Mode.WORK:
            return c.track_map.get(c.target.value)
        return None

    def _reenter(self, state: State | None) -> list[Action]:
        """멈춘 동작을 다시 시작한다 (비상 정지·오류·장애물 뒤)."""
        c = self.ctx
        color = self._route_color()                         # 재연결 때 앱이 목적지 색을 바꿨을 수 있음 (교수 점검 2.2)
        again = [A("TARGET", color=color)] if color else []
        if state == State.UTURN:
            self.state = State.UTURN
            return [A("POSE", pose=Pose.LINE.value)] + again + [A("DRIVE", command=DriveCommand.UTURN.value, skip=0)]
        if state == State.NAVIGATE:
            return [A("POSE", pose=Pose.LINE.value)] + again + self._navigate()
        if state == State.APPROACH:
            return self._aim()
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

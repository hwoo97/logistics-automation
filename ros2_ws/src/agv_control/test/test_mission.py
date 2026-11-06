"""MissionStateMachine — 가짜 트랙(색 → 지점)에서 동작을 흉내 내며 초기 설정·작업 흐름을 끝까지 돌린다."""
import pytest

from agv_control.mission import Event, MissionConfig, MissionStateMachine
from agv_protocol.types import Cargo, Mode, State

TRACK = {"red": "DROP_A", "green": "DROP_B", "blue": "WORK", "yellow": "PARK"}   # 실제 배치 (차량은 모름)
SIGN = {"WORK": "sign_work", "DROP_A": "sign_drop_a", "DROP_B": "sign_drop_b", "PARK": "sign_park"}
FULL = {"WORK": "blue", "DROP_A": "red", "DROP_B": "green", "PARK": "yellow"}


class Keep:
    """입력 순서 그대로 (원 → 사각) — 무작위 순서를 고정해 시험."""
    def shuffle(self, items):
        pass


class Reverse:
    def shuffle(self, items):
        items.reverse()


class World:
    """동작을 받아 다음 이벤트를 돌려주는 가짜 차량·트랙. 차량은 처음에 대기장소 표지판 앞."""

    def __init__(self, sm: MissionStateMachine, track=TRACK, sign_fail: int = 0, facing_sign: bool = True):
        self.sm, self.track, self.sign_fail = sm, track, sign_fail
        self.facing_sign = facing_sign                  # 처음에 대기장소 표지판을 보고 있는지 (초음파 0.3m / 값 없음)
        self.at, self.color, self.now = "PARK", "NONE", 0.0
        self.log, self.digits, self.leds, self.reports, self.saved = [], [], [], [], None
        self.signs = []
        self.queue: list[Event] = []

    def send(self, kind, **data):
        if kind in ("SETUP", "START"):                   # 출발 전 초음파 — 표지판을 보고 있으면 0.3m
            self.queue.append(Event("RANGE", {"distance": 0.3 if self.facing_sign else None}))
        self.queue.append(Event(kind, data))
        return self.run()

    def run(self, limit=2000):
        steps = 0
        while self.queue and steps < limit:
            ev = self.queue.pop(0)
            ev.data.setdefault("now", self.now)
            for act in self.sm.handle(ev):
                self.apply(act)
            steps += 1
            if not self.queue and self.sm.state in (State.LOAD, State.UNLOAD, State.CHECK_SIGN):
                self.now += 0.5
                self.queue.append(Event("TICK", {"now": self.now}))
        return self

    def apply(self, act):
        k, d = act.kind, act.data
        self.log.append((k, dict(d)))
        if k == "TARGET":
            self.color = d["color"]
        elif k == "DIGIT":
            self.digits.append(d["char"])
        elif k == "LED":
            self.leds.append(d["state"])
        elif k == "REPORT":
            self.reports.append(d.get("event") or d.get("error"))
            if d.get("event") == "SIGN":
                self.signs.append(d.get("detail", ""))
        elif k == "SAVE_MAP":
            self.saved = dict(d["track_map"])
        elif k == "POSE":
            self.queue.append(Event("POSE_READY", {"pose": d["pose"]}))
        elif k == "CLASSIFY":
            if self.sign_fail:
                self.sign_fail -= 1
                self.queue.append(Event("SIGN", {"label": ""}))
            else:
                self.queue.append(Event("SIGN", {"label": SIGN[self.at]}))
        elif k == "DRIVE":
            cmd = d["command"]
            if cmd == "UTURN":
                self.queue.append(Event("DRIVE_DONE", {"result": "UTURN_DONE"}))
            elif cmd == "FOLLOW_TO_EVENT":
                assert d["skip"] == 1                          # 유턴 뒤에는 늘 방금 지점의 정지선 통과
                dest = self.track.get(self.color)
                if dest is None:
                    self.queue.append(Event("DRIVE_DONE", {"result": "LINE_END"}))
                else:
                    self.at = dest
                    self.queue.append(Event("DRIVE_DONE", {"result": "STOP_LINE"}))
            elif cmd == "APPROACH":
                for dist in (0.4, 0.3, 0.2, 0.18, 0.17, 0.17):
                    self.queue.append(Event("RANGE", {"distance": dist}))


def sm(**cfg):
    return MissionStateMachine(MissionConfig(**cfg))


def test_setup_maps_all_colors_and_returns_to_park():
    w = World(sm()).send("SETUP")
    assert w.sm.state == State.PARKED and w.at == "PARK"
    assert w.sm.ctx.track_map == FULL and w.saved == FULL and w.sm.mapped
    assert [r for r in w.reports if r == "MAPPED"] == ["MAPPED"] * 4 and w.reports[-1] == "MAP_DONE"
    # 빨강(DROP_A)·파랑(WORK)·초록(DROP_B)을 읽고 노랑(PARK)은 소거법 → 대기장소로 가서 확인
    mapped = [d["detail"] for k, d in w.log if k == "REPORT" and d.get("event") == "MAPPED"]
    assert mapped == ["DROP_A red", "WORK blue", "DROP_B green", "PARK yellow 소거"]
    assert w.digits[0] == "0" and w.digits[-1] == "0"           # 빈 차량은 0 (변경 시나리오)
    w.send("TICK", now=w.now + 5)
    assert w.sm.state == State.IDLE


def test_setup_returns_to_park_when_last_color_is_not_park():
    track = {"red": "PARK", "green": "DROP_B", "blue": "WORK", "yellow": "DROP_A"}
    w = World(sm(), track).send("SETUP")
    assert w.sm.state == State.PARKED and w.at == "PARK"
    assert w.sm.ctx.track_map == {"PARK": "red", "DROP_B": "green", "WORK": "blue", "DROP_A": "yellow"}
    assert w.color == "NONE"                                     # 끝나면 목적지 색 해제
    follows = [d for k, d in w.log if k == "TARGET"]
    # 빨강·파랑·초록만 따라가고 노랑(DROP_A)은 소거법 → 대기장소(빨강)로 복귀. 노랑 지점은 가지 않음
    assert [d["color"] for d in follows][:5] == ["red", "blue", "green", "red", "NONE"]


def test_setup_does_not_check_start_sign():
    """2026-11-06: 출발할 때 표지판을 읽지 않음. 앞이 가까우면(표지판을 보고 있음) 유턴 → 빨강부터 따라감."""
    m = sm()
    m.handle(Event("RANGE", {"distance": 0.3}))
    acts = m.handle(Event("SETUP"))
    assert m.state == State.UTURN and not any(a.kind == "CLASSIFY" for a in acts)
    assert [a.data["color"] for a in acts if a.kind == "TARGET"] == ["red"]
    assert ("DRIVE", "UTURN") in kinds(acts)
    assert [a.data["detail"] for a in acts if a.kind == "REPORT" and a.data.get("event") == "DEPART"] == ["UTURN 앞 0.30m"]


@pytest.mark.parametrize("dist", [None, 0.8, 1.9])
def test_setup_facing_away_starts_without_uturn(dist):
    """처음 방향을 모름 — 앞이 멀거나 값이 없으면(바깥을 보고 있음) 유턴 없이 바로 라인을 따라감, 대기장소 정지선은 통과."""
    m = sm()
    m.handle(Event("RANGE", {"distance": dist}))
    acts = m.handle(Event("SETUP"))
    assert m.state == State.NAVIGATE and ("DRIVE", "UTURN") not in kinds(acts)
    drive = [a for a in acts if a.kind == "DRIVE"][0]
    assert drive.data == {"command": "FOLLOW_TO_EVENT", "skip": 1}
    assert [a.data["color"] for a in acts if a.kind == "TARGET"] == ["red"]


def test_setup_whole_run_facing_away():
    w = World(sm(), facing_sign=False).send("SETUP")
    assert w.sm.state == State.PARKED and w.sm.ctx.track_map == FULL
    first_drive = [d for k, d in w.log if k == "DRIVE"][0]
    assert first_drive["command"] == "FOLLOW_TO_EVENT"


def test_work_start_facing_away_starts_without_uturn():
    m = MissionStateMachine(track_map=FULL)
    m.handle(Event("RANGE", {"distance": 1.2}))
    acts = m.handle(Event("START", {"circle": 1, "square": 0}))
    assert m.state == State.NAVIGATE and ("DRIVE", "UTURN") not in kinds(acts)


def test_later_trips_always_uturn():
    """출발 뒤의 이동(지점 → 다음 지점)은 늘 표지판 앞에서 시작하므로 유턴 (초음파와 상관없음)."""
    w = World(sm(), facing_sign=False).send("SETUP")
    drives = [d["command"] for k, d in w.log if k == "DRIVE"]
    assert drives[0] == "FOLLOW_TO_EVENT" and drives.count("UTURN") >= 3


def test_setup_order_red_blue_green_yellow_and_elimination_last():
    """빨강→파랑→초록 순서로 세 곳을 읽으면 노랑은 소거법 — 노랑 지점은 가지 않음."""
    w = World(sm()).send("SETUP")
    follows = [d["color"] for k, d in w.log if k == "TARGET"]
    assert follows[:3] == ["red", "blue", "green"] and "yellow" in follows[3:4]   # 넷째는 대기장소(노랑) 복귀
    assert w.signs.count("sign_park 0.00") == 1                  # 대기장소 표지판은 복귀 때 한 번만 읽음


def test_setup_park_found_early_still_eliminates_last():
    """대기장소가 세 곳 안에 있으면 남은 하차·상차 지점이 소거법, 마지막으로 읽은 곳이 대기장소가 아니면 대기장소로 복귀."""
    track = {"red": "WORK", "blue": "PARK", "green": "DROP_A", "yellow": "DROP_B"}
    w = World(sm(), track).send("SETUP")
    assert w.sm.state == State.PARKED and w.at == "PARK"
    assert w.sm.ctx.track_map == {"WORK": "red", "PARK": "blue", "DROP_A": "green", "DROP_B": "yellow"}
    mapped = [d["detail"] for k, d in w.log if k == "REPORT" and d.get("event") == "MAPPED"]
    assert mapped[-1] == "DROP_B yellow 소거"


def test_setup_third_is_park_finishes_there():
    """셋째로 읽은 곳이 대기장소면 넷째를 소거법으로 채우고 그 자리에서 끝."""
    track = {"red": "WORK", "blue": "DROP_A", "green": "PARK", "yellow": "DROP_B"}
    w = World(sm(), track).send("SETUP")
    assert w.sm.state == State.PARKED and w.at == "PARK"
    follows = [d["color"] for k, d in w.log if k == "TARGET"]
    assert follows == ["red", "blue", "green", "NONE"]


def test_setup_duplicate_point_is_error():
    track = {"red": "DROP_A", "green": "DROP_A", "blue": "WORK", "yellow": "PARK"}   # 초록 스티커가 잘못 붙음
    w = World(sm(), track).send("SETUP")
    assert w.sm.state == State.ERROR and w.reports[-1] == "E_MAP" and "DROP_A" in w.sm.ctx.error


def test_setup_missing_marker_line_end_is_error():
    track = {"red": "DROP_A", "blue": "WORK", "yellow": "PARK"}                      # 초록 스티커 없음
    w = World(sm(), track).send("SETUP")
    assert w.sm.state == State.ERROR and w.reports[-1] == "E_LINE"


def test_start_needs_full_map():
    w = World(sm()).send("START", circle=1, square=0)
    assert w.sm.state == State.IDLE and w.reports == ["E_MAP"]


def work(circle, square, **cfg):
    m = MissionStateMachine(MissionConfig(**cfg), track_map=FULL, rng=Keep())
    return World(m).send("START", circle=circle, square=square)


def test_work_circle_then_square_and_back():
    w = work(3, 2)
    assert w.sm.state == State.PARKED and w.at == "PARK"
    assert [r for r in w.reports if r not in ("SIGN", "DEPART")] == ["STARTED", "LOADED", "UNLOADED", "LOADED", "UNLOADED",
                                                                    "PARKED"]
    assert w.digits == ["0", "1", "2", "3", "2", "1", "0", "1", "2", "1", "0", "0"]
    assert w.leds[1:] == ["RED", "OFF", "GREEN", "OFF", "OFF"] and w.sm.ctx.done == 5
    colors = [d["color"] for k, d in w.log if k == "TARGET"]
    assert colors == ["blue", "red", "blue", "green", "yellow", "NONE"]


def test_work_digits_count_up_and_down():
    w = work(0, 3)
    assert w.digits == ["0", "1", "2", "3", "2", "1", "0", "0"]
    assert [d["color"] for k, d in w.log if k == "TARGET"] == ["blue", "green", "yellow", "NONE"]


def test_one_item_per_second():
    m2 = MissionStateMachine(track_map=FULL)
    m2.state = State.LOAD
    m2.ctx.plan, m2.ctx.cargo, m2.ctx.next_tick = [(Cargo.CIRCLE, 2)], Cargo.CIRCLE, 1.0
    assert m2.handle(Event("TICK", {"now": 0.9})) == []
    assert [a.data["char"] for a in m2.handle(Event("TICK", {"now": 1.0})) if a.kind == "DIGIT"] == ["1"]
    assert m2.handle(Event("TICK", {"now": 1.5})) == []
    assert [a.data["char"] for a in m2.handle(Event("TICK", {"now": 2.0})) if a.kind == "DIGIT"] == ["2"]


def test_wrong_sign_in_work_is_error():
    track = dict(TRACK, red="DROP_B", green="DROP_A")           # 매핑과 트랙이 다름 (스티커를 바꿔 붙임)
    m = MissionStateMachine(track_map=FULL)
    w = World(m, track).send("START", circle=1, square=0)
    assert w.sm.state == State.ERROR and w.reports[-1] == "E_SIGN"
    assert w.leds[-1] == "ERROR" and w.digits[-1] == "E"


def test_sign_retry_then_ok():
    m = MissionStateMachine(track_map=FULL)
    w = World(m, sign_fail=1).send("START", circle=1, square=0)
    assert w.sm.state == State.PARKED


def test_sign_fail_twice_is_error_and_resume_retries():
    m = MissionStateMachine(track_map=FULL)
    w = World(m, sign_fail=2).send("START", circle=1, square=0)
    assert w.sm.state == State.ERROR and w.reports[-1] == "E_SIGN"
    w.send("RESUME")
    assert w.sm.state == State.PARKED


def test_approach_timeout():
    m = MissionStateMachine(MissionConfig(approach_timeout_s=10), track_map=FULL)
    m.state, m.ctx.since = State.APPROACH, 0.0
    acts = m.handle(Event("TICK", {"now": 10.5}))
    assert m.state == State.ERROR and any(a.kind == "CANCEL_DRIVE" for a in acts)
    assert any(a.data.get("error") == "E_APPROACH" for a in acts)


def test_approach_needs_consecutive_close_readings():
    m = MissionStateMachine(track_map=FULL)
    m.state = State.APPROACH
    for d in (0.1, 0.5, 0.1, 0.1):
        m.handle(Event("RANGE", {"distance": d}))
    assert m.state == State.APPROACH
    acts = m.handle(Event("RANGE", {"distance": 0.1}))
    assert m.state == State.CHECK_SIGN and [a.kind for a in acts] == ["CANCEL_DRIVE", "POSE"]


def test_obstacle_ignored_by_default():
    """변경 시나리오: 장애물 없음 — 가까운 값이 와도 주행을 멈추지 않음."""
    m = MissionStateMachine(track_map=FULL)
    m.state = State.NAVIGATE
    assert m.handle(Event("RANGE", {"distance": 0.05})) == [] and m.state == State.NAVIGATE


def test_obstacle_pauses_and_resumes_navigate():
    m = MissionStateMachine(MissionConfig(obstacle_enabled=True), track_map=FULL)
    m.state, m.ctx.skip_stop_lines = State.NAVIGATE, 1
    acts = m.handle(Event("RANGE", {"distance": 0.1, "now": 0.0}))
    assert m.state == State.PAUSED_OBSTACLE and acts[0].kind == "CANCEL_DRIVE"
    m.handle(Event("RANGE", {"distance": 0.5, "now": 0.2}))
    m.handle(Event("RANGE", {"distance": 0.1, "now": 0.5}))      # 다시 가까워지면 처음부터
    m.handle(Event("RANGE", {"distance": 0.5, "now": 0.6}))
    assert m.state == State.PAUSED_OBSTACLE
    acts = m.handle(Event("RANGE", {"distance": 0.5, "now": 1.7}))
    drive = [a for a in acts if a.kind == "DRIVE"][0]
    assert m.state == State.NAVIGATE and drive.data == {"command": "FOLLOW_TO_EVENT", "skip": 1}


def test_drive_obstacle_result_pauses_and_resumes():
    """drive_node 초음파 정지(OBSTACLE) — 오류가 아니라 장애물 대기, 0.18m보다 멀어진 채 1초면 주행 재개."""
    m = MissionStateMachine(track_map=FULL)
    m.state, m.ctx.skip_stop_lines = State.NAVIGATE, 1
    acts = m.handle(Event("DRIVE_DONE", {"result": "OBSTACLE", "now": 0.0}))
    assert m.state == State.PAUSED_OBSTACLE
    assert [a.kind for a in acts] == ["REPORT"] and acts[0].data["event"] == "OBSTACLE"
    m.handle(Event("RANGE", {"distance": 0.17, "now": 0.1}))
    m.handle(Event("RANGE", {"distance": 0.3, "now": 0.2}))
    assert m.state == State.PAUSED_OBSTACLE
    acts = m.handle(Event("RANGE", {"distance": 0.3, "now": 1.3}))
    drive = [a for a in acts if a.kind == "DRIVE"][0]
    assert m.state == State.NAVIGATE and drive.data == {"command": "FOLLOW_TO_EVENT", "skip": 1}


def test_drive_obstacle_then_estop_resume_continues_navigate():
    m = MissionStateMachine(track_map=FULL)
    m.state = State.NAVIGATE
    m.handle(Event("DRIVE_DONE", {"result": "OBSTACLE"}))
    m.handle(Event("STOP"))
    acts = m.handle(Event("RESUME"))
    assert m.state == State.NAVIGATE and any(a.data.get("command") == "FOLLOW_TO_EVENT" for a in acts)


def test_passed_stop_line_is_not_skipped_again_after_obstacle():
    m = MissionStateMachine(track_map=FULL)
    m.state, m.ctx.skip_stop_lines = State.NAVIGATE, 1
    m.handle(Event("PHASE", {"phase": "PASS STOP_LINE"}))
    m.handle(Event("PHASE", {"phase": "FOLLOW"}))
    assert m.ctx.skip_stop_lines == 0


def test_estop_and_resume_continue_where_stopped():
    m2 = MissionStateMachine(track_map=FULL)
    m2.state = State.NAVIGATE
    acts = m2.handle(Event("STOP"))
    assert m2.state == State.ESTOP and [a.kind for a in acts] == ["ESTOP", "CANCEL_DRIVE"]
    acts = m2.handle(Event("RESUME"))
    assert m2.state == State.NAVIGATE and acts[0].data == {"on": False}
    assert any(a.kind == "DRIVE" and a.data["command"] == "FOLLOW_TO_EVENT" for a in acts)


def test_estop_during_obstacle_resumes_previous_drive():
    m = MissionStateMachine(MissionConfig(obstacle_enabled=True), track_map=FULL)
    m.state = State.UTURN
    m.handle(Event("RANGE", {"distance": 0.1}))
    m.handle(Event("STOP"))
    acts = m.handle(Event("RESUME"))
    assert m.state == State.UTURN and any(a.data.get("command") == "UTURN" for a in acts)


def test_estop_during_load_keeps_counting_after_resume():
    m = MissionStateMachine(track_map=FULL)
    m.state, m.ctx.mode = State.LOAD, Mode.WORK
    m.ctx.plan, m.ctx.cargo, m.ctx.load = [(Cargo.CIRCLE, 3)], Cargo.CIRCLE, 2
    m.handle(Event("STOP"))
    acts = m.handle(Event("RESUME", {"now": 10.0}))
    assert m.state == State.LOAD and m.ctx.next_tick == pytest.approx(11.0)
    assert {"state": "RED"} in [a.data for a in acts if a.kind == "LED"]
    assert {"char": "2"} in [a.data for a in acts if a.kind == "DIGIT"]


def test_manual_from_idle_and_move():
    m = MissionStateMachine()
    m.handle(Event("MANUAL_ON"))
    assert m.state == State.MANUAL
    assert m.handle(Event("MOVE", {"linear": 0.3, "angular": 0.1}))[0].data == {"linear": 0.3, "angular": 0.1}
    m.handle(Event("MANUAL_OFF"))
    assert m.state == State.IDLE and m.handle(Event("MOVE", {"linear": 0.3, "angular": 0})) == []


def test_commands_ignored_outside_idle():
    m = MissionStateMachine(track_map=FULL)
    m.state = State.NAVIGATE
    assert m.handle(Event("START", {"circle": 1, "square": 0})) == [] and m.handle(Event("SETUP")) == []


def test_status_fields():
    m = MissionStateMachine(track_map=FULL)
    w = World(m)
    w.send("START", circle=2, square=0)
    s = m.status()
    assert s["mode"] == "WORK" and s["total"] == 2 and s["map"] == FULL
    m2 = MissionStateMachine()
    m2.handle(Event("RANGE", {"distance": 0.3}))
    m2.handle(Event("SETUP"))
    s = m2.status()
    assert s["mode"] == "SETUP" and s["total"] == 4 and s["state"] == "UTURN" and s["target"] == "red"


# ── 변경 시나리오 2026-11-03 ─────────────────────────────

def test_plan_splits_into_nine_and_random_first():
    m = MissionStateMachine(track_map=FULL, rng=Keep())
    assert m.make_plan(12, 5) == [(Cargo.CIRCLE, 9), (Cargo.CIRCLE, 3), (Cargo.SQUARE, 5)]
    m = MissionStateMachine(track_map=FULL, rng=Reverse())
    assert m.make_plan(12, 5) == [(Cargo.SQUARE, 5), (Cargo.CIRCLE, 9), (Cargo.CIRCLE, 3)]
    assert m.make_plan(0, 18) == [(Cargo.SQUARE, 9), (Cargo.SQUARE, 9)] and m.make_plan(0, 0) == []


def test_random_order_uses_both_orders():
    import random
    firsts = {MissionStateMachine(track_map=FULL, rng=random.Random(seed)).make_plan(3, 3)[0][0] for seed in range(20)}
    assert firsts == {Cargo.CIRCLE, Cargo.SQUARE}


def test_professor_example_red12_green5_green_first():
    """전달사항 5절 예시: 적 12개, 녹 5개, 녹이 먼저 → 녹 5, 적 9, 적 3 순서로 왕복 후 Start 복귀."""
    m = MissionStateMachine(track_map=FULL, rng=Reverse())
    w = World(m).send("START", circle=12, square=5)
    assert w.sm.state == State.PARKED and w.at == "PARK" and m.ctx.done == 17 and m.ctx.total == 17
    loads = [r for r in w.reports if r in ("LOADED", "UNLOADED")]
    assert loads == ["LOADED", "UNLOADED"] * 3
    details = [d["detail"] for k, d in w.log if k == "REPORT" and d.get("event") in ("LOADED", "UNLOADED")]
    assert details == ["SQUARE 5", "SQUARE 5", "CIRCLE 9", "CIRCLE 9", "CIRCLE 3", "CIRCLE 3"]
    assert w.leds[1:] == ["GREEN", "OFF", "RED", "OFF", "RED", "OFF", "OFF"]
    colors = [d["color"] for k, d in w.log if k == "TARGET"]
    assert colors == ["blue", "green", "blue", "red", "blue", "red", "yellow", "NONE"]
    peaks = [int(c) for c in w.digits]
    assert max(peaks) == 9 and w.digits[-1] == "0"


def test_sign_distance_is_30cm():
    """2026-11-06 사용자 결정: 표지판 30cm 앞에서 멈춤 (변경 시나리오 18cm에서 바꿈)."""
    m = MissionStateMachine(track_map=FULL)
    m.state = State.APPROACH
    for _ in range(3):
        m.handle(Event("RANGE", {"distance": 0.31}))
    assert m.state == State.APPROACH
    for _ in range(3):
        m.handle(Event("RANGE", {"distance": 0.30}))
    assert m.state == State.CHECK_SIGN


def test_classify_waits_one_second_after_stop():
    """멈추고 카메라 자세가 잡힌 뒤 1초 기다렸다 분류."""
    m = MissionStateMachine(track_map=FULL)
    m.state, m.ctx.aiming = State.APPROACH, False
    for _ in range(3):
        m.handle(Event("RANGE", {"distance": 0.25, "now": 5.0}))
    assert m.state == State.CHECK_SIGN
    assert m.handle(Event("POSE_READY", {"pose": "SIGN", "now": 5.1})) == []
    assert m.handle(Event("TICK", {"now": 5.6})) == []
    assert m.handle(Event("TICK", {"now": 6.0})) == []
    acts = m.handle(Event("TICK", {"now": 6.1}))
    assert [a.kind for a in acts] == ["CLASSIFY"]
    assert m.handle(Event("TICK", {"now": 6.5})) == []          # 한 번만


def test_classify_retry_does_not_wait_again():
    m = MissionStateMachine(track_map=FULL)
    m.state, m.ctx.test = State.CHECK_SIGN, True
    acts = m.handle(Event("SIGN", {"label": "", "message": "확정 못 함", "now": 7.0}))
    assert [a.kind for a in acts] == ["REPORT", "CLASSIFY"]


def test_classify_without_wait_when_settle_zero():
    m = MissionStateMachine(MissionConfig(sign_settle_s=0.0), track_map=FULL)
    m.state = State.CHECK_SIGN
    assert [a.kind for a in m.handle(Event("POSE_READY", {"pose": "SIGN"}))] == ["CLASSIFY"]


# ── 도착 순서 (교수 변경 시나리오: 정지선 → 카메라 표지판 쪽 → 전진 → 18cm 정지) ──

def kinds(acts):
    return [(a.kind, a.data.get("pose") or a.data.get("command")) for a in acts]


def test_stop_line_raises_camera_before_approach():
    m = MissionStateMachine(track_map=FULL)
    m.state = State.NAVIGATE
    acts = m.handle(Event("DRIVE_DONE", {"result": "STOP_LINE"}))
    assert m.state == State.APPROACH and kinds(acts) == [("POSE", "SIGN")]       # 아직 주행하지 않음
    assert m.handle(Event("RANGE", {"distance": 0.1})) == []                      # 카메라 올리는 중 초음파 무시
    acts = m.handle(Event("POSE_READY", {"pose": "SIGN"}))
    assert kinds(acts) == [("DRIVE", "APPROACH")]
    for _ in range(3):
        acts = m.handle(Event("RANGE", {"distance": 0.17}))
    assert m.state == State.CHECK_SIGN and acts[0].kind == "CANCEL_DRIVE"


def test_full_work_trip_raises_camera_at_each_stop_line():
    w = World(sm()).send("SETUP")
    seq = [(k, d.get("pose") or d.get("command")) for k, d in w.log if k in ("POSE", "DRIVE")]
    for i, item in enumerate(seq):
        if item == ("DRIVE", "APPROACH"):
            assert seq[i - 1] == ("POSE", "SIGN")


def test_resume_in_approach_raises_camera_again():
    m = MissionStateMachine(track_map=FULL)
    m.state = State.NAVIGATE
    m.handle(Event("DRIVE_DONE", {"result": "STOP_LINE"}))
    m.handle(Event("POSE_READY", {"pose": "SIGN"}))
    m.handle(Event("STOP"))
    acts = m.handle(Event("RESUME"))
    assert m.state == State.APPROACH and ("POSE", "SIGN") in kinds(acts) and ("DRIVE", "APPROACH") not in kinds(acts)


# ── 도착 시험 (개발자 탭) ──

def arrive_test(label="sign_work", **cfg):
    m = MissionStateMachine(MissionConfig(**cfg))
    acts = m.handle(Event("TEST_ARRIVE"))
    assert m.state == State.NAVIGATE and ("DRIVE", "FOLLOW_TO_EVENT") in kinds(acts)
    drive = [a for a in acts if a.kind == "DRIVE"][0]
    assert drive.data["skip"] == 0 and not any(a.kind == "TARGET" for a in acts)   # 목적지 색은 개발자 탭 값 그대로
    m.handle(Event("DRIVE_DONE", {"result": "STOP_LINE", "now": 0.0}))
    m.handle(Event("POSE_READY", {"pose": "SIGN", "now": 0.5}))
    for _ in range(3):
        m.handle(Event("RANGE", {"distance": 0.15, "now": 2.0}))
    assert m.handle(Event("POSE_READY", {"pose": "SIGN", "now": 2.1})) == []      # 1초 기다림
    acts = m.handle(Event("TICK", {"now": 3.2}))
    assert [a.kind for a in acts] == ["CLASSIFY"]
    return m


def test_arrive_test_reports_sign_and_lowers_camera():
    m = arrive_test()
    acts = m.handle(Event("SIGN", {"label": "sign_work"}))
    assert m.state == State.IDLE and ("POSE", "LINE") in kinds(acts)
    reports = [a.data for a in acts if a.kind == "REPORT"]
    assert reports == [{"event": "SIGN", "detail": "sign_work 0.00"}, {"event": "ARRIVED", "detail": "sign_work"}]


def test_arrive_test_without_model_is_not_error():
    m = arrive_test()
    m.handle(Event("SIGN", {"label": "", "message": "모델 없음"}))                # 다시 한 번 봄
    acts = m.handle(Event("SIGN", {"label": "", "message": "모델 없음"}))
    assert m.state == State.IDLE
    assert [a.data for a in acts if a.kind == "REPORT"] == [{"event": "SIGN", "detail": "- 모델 없음"},
                                                            {"event": "ARRIVED", "detail": "표지판 못 읽음 모델 없음"}]


def test_arrive_test_only_from_idle():
    m = MissionStateMachine(track_map=FULL)
    m.state = State.NAVIGATE
    assert m.handle(Event("TEST_ARRIVE")) == []


def test_cancel_stops_and_goes_idle():
    m = MissionStateMachine()
    m.handle(Event("TEST_ARRIVE"))
    acts = m.handle(Event("CANCEL"))
    assert m.state == State.IDLE and acts[0].kind == "CANCEL_DRIVE" and not m.ctx.test
    assert m.handle(Event("CANCEL")) == []                                       # IDLE이면 아무것도 안 함


def test_sign_result_reported_with_score():
    """표지판 분류 결과는 매번 EVENT SIGN <라벨> <점수> — 개발자 탭 "표지판 인식" 칸."""
    m = MissionStateMachine(track_map=FULL)
    m.state = State.CHECK_SIGN
    m.ctx.test = True
    acts = m.handle(Event("SIGN", {"label": "sign_drop_a", "score": 0.934}))
    assert acts[0].kind == "REPORT" and acts[0].data == {"event": "SIGN", "detail": "sign_drop_a 0.93"}


def test_sign_failure_reported_with_reason():
    m = MissionStateMachine(track_map=FULL)
    m.state = State.CHECK_SIGN
    m.ctx.test = True
    acts = m.handle(Event("SIGN", {"label": "", "message": "확정 못 함 (5장: none 0.99)"}))
    assert acts[0].data == {"event": "SIGN", "detail": "- 확정 못 함 (5장: none 0.99)"}
    assert [a.kind for a in acts[1:]] == ["CLASSIFY"]                # 재시도는 그대로


def test_sign_reported_in_work_too():
    w = work(1, 1)
    assert w.signs and all(s.split()[0].startswith("sign_") for s in w.signs)


def test_sign_ignored_outside_check_sign():
    m = MissionStateMachine(track_map=FULL)
    m.state = State.NAVIGATE
    assert m.handle(Event("SIGN", {"label": "sign_work", "score": 0.9})) == []

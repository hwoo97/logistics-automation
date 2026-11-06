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

    def __init__(self, sm: MissionStateMachine, track=TRACK, sign_fail: int = 0):
        self.sm, self.track, self.sign_fail = sm, track, sign_fail
        self.at, self.color, self.now = "PARK", "NONE", 0.0
        self.log, self.digits, self.leds, self.reports, self.saved = [], [], [], [], None
        self.queue: list[Event] = []

    def send(self, kind, **data):
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
            if not self.queue and self.sm.state in (State.LOAD, State.UNLOAD):
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
    assert [d["color"] for d in follows][:5] == ["red", "green", "blue", "yellow", "red"]


def test_setup_rejects_wrong_start_position():
    w = World(sm())
    w.at = "WORK"
    w.send("SETUP")
    assert w.sm.state == State.ERROR and w.reports[-1] == "E_MAP"


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
    assert w.reports == ["STARTED", "LOADED", "UNLOADED", "LOADED", "UNLOADED", "PARKED"]
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
    m2.handle(Event("SETUP"))
    s = m2.status()
    assert s["mode"] == "SETUP" and s["total"] == 4 and s["state"] == "CHECK_SIGN"


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


def test_sign_distance_is_18cm():
    m = MissionStateMachine(track_map=FULL)
    m.state = State.APPROACH
    for _ in range(3):
        m.handle(Event("RANGE", {"distance": 0.19}))
    assert m.state == State.APPROACH
    for _ in range(3):
        m.handle(Event("RANGE", {"distance": 0.18}))
    assert m.state == State.CHECK_SIGN

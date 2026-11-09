"""mission_node — MissionStateMachine을 ROS에 연결 (F12, functional_design 5절).

입력: /mission/command(Command), /ultrasonic/range, /pantilt/ready, /drive 액션 결과·피드백, /vision/classify 응답, 10Hz 타이머
출력: /drive 액션, /drive/target, /pantilt/pose, /led/state, /display/digit, /safety/estop, /cmd_vel(수동),
      /mission/status(Status, 2Hz + 바뀔 때), /mission/report(블루투스 EVENT·ERROR 한 줄 — 브리지가 그대로 보냄)
트랙 매핑은 map_file(기본 ~/.agv/track_map.json)에 저장해 다시 켜도 유지한다.
"""
import json
import os
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String

from agv_control.mission import Event, MissionConfig, MissionStateMachine
from agv_interfaces.action import Drive
from agv_interfaces.msg import Command, Status
from agv_interfaces.srv import Classify
from agv_protocol.messages import format_error, format_event


def load_map(path: str) -> dict:
    try:
        with open(path, encoding="utf-8") as fp:
            data = json.load(fp)
        return {k: v for k, v in data.items() if isinstance(v, str)}
    except (OSError, ValueError, AttributeError):
        return {}


def save_map(path: str, track_map: dict) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as fp:
        json.dump(track_map, fp, ensure_ascii=False, indent=2)


class MissionNode(Node):
    def __init__(self):
        super().__init__("mission_node")
        p = self.declare_parameter
        d = MissionConfig()
        cfg = MissionConfig(**{k: p(k, getattr(d, k)).value for k in d.__dataclass_fields__})
        self.map_file = os.path.expanduser(p("map_file", "~/.agv/track_map.json").value)
        self.lock = threading.RLock()
        self.sm = MissionStateMachine(cfg, load_map(self.map_file))
        self.goal = None                                     # 진행 중 /drive 목표 핸들
        self.goal_seq = 0                                    # 오래된 목표의 결과를 버리려고 번호를 붙임
        self.last_status = None

        group = ReentrantCallbackGroup()
        q10 = QoSProfile(depth=10)
        self.pub_status = self.create_publisher(Status, "/mission/status", QoSProfile(depth=1))
        self.pub_report = self.create_publisher(String, "/mission/report", q10)
        self.pub_pose = self.create_publisher(String, "/pantilt/pose", q10)
        self.pub_led = self.create_publisher(String, "/led/state", q10)
        self.pub_digit = self.create_publisher(String, "/display/digit", q10)
        self.pub_target = self.create_publisher(String, "/drive/target", q10)
        self.pub_estop = self.create_publisher(Bool, "/safety/estop", q10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", QoSProfile(depth=1))
        self.create_subscription(Command, "/mission/command", self.on_command, q10, callback_group=group)
        self.create_subscription(Range, "/ultrasonic/range", self.on_range, qos_profile_sensor_data, callback_group=group)
        self.create_subscription(String, "/pantilt/ready", self.on_ready, q10, callback_group=group)
        self.create_subscription(String, "/mission/tune", self.on_tune, q10, callback_group=group)
        self.drive = ActionClient(self, Drive, "/drive", callback_group=group)
        self.classify = self.create_client(Classify, "/vision/classify", callback_group=group)
        self.create_timer(0.1, self.on_tick, callback_group=group)
        self.create_timer(0.5, lambda: self.publish_status(force=True), callback_group=group)
        self.get_logger().info(f"mission_node 시작 (매핑 {self.sm.ctx.track_map or '없음'}, 파일 {self.map_file})")

    # ── 입력 → 이벤트 ────────────────────────────────────

    def feed(self, kind: str, **data):
        with self.lock:
            data.setdefault("now", time.monotonic())
            actions = self.sm.handle(Event(kind, data))
            for act in actions:
                self.execute(act)
            self.publish_status()

    def on_command(self, msg: Command):
        kind = msg.kind
        if kind == "START":
            self.feed("START", circle=int(msg.circle), square=int(msg.square))
        elif kind == "MOVE":
            self.feed("MOVE", linear=float(msg.linear), angular=float(msg.angular))
        else:
            self.feed(kind)                                  # SETUP STOP RESUME MANUAL_ON MANUAL_OFF

    def on_tune(self, msg: String):
        """노트북 앱 "표지판 정지 거리" (DEV TUNE SIGN, m) — 실험하며 조정 (2026-11-09). 다음 접근부터 적용."""
        try:
            key, value = msg.data.split()
            if key != "SIGN":
                raise ValueError(f"모르는 항목 {key}")
            with self.lock:
                self.sm.cfg.sign_distance = float(value)
        except ValueError as e:
            self.get_logger().warn(f"/mission/tune 형식 오류 {msg.data!r}: {e}")
            return
        self.get_logger().info(f"조정 표지판 정지 거리 = {float(value):.2f}m")

    def on_range(self, msg: Range):
        r = msg.range
        self.feed("RANGE", distance=r if r == r and r != float("inf") else None)

    def on_ready(self, msg: String):
        self.feed("POSE_READY", pose=msg.data)

    def on_tick(self):
        self.feed("TICK")

    # ── 동작 실행 ─────────────────────────────────────────

    def execute(self, act):
        k, d = act.kind, act.data
        if k == "DRIVE":
            self.send_drive(d["command"], d.get("skip", 0))
        elif k == "CANCEL_DRIVE":
            self.cancel_drive()
        elif k == "TARGET":
            self.pub_target.publish(String(data=d["color"]))
        elif k == "POSE":
            self.pub_pose.publish(String(data=d["pose"]))
        elif k == "LED":
            self.pub_led.publish(String(data=d["state"]))
        elif k == "DIGIT":
            self.pub_digit.publish(String(data=d["char"]))
        elif k == "ESTOP":
            self.pub_estop.publish(Bool(data=bool(d["on"])))
        elif k == "CMD_VEL":
            msg = Twist()
            msg.linear.x, msg.angular.z = float(d["linear"]), float(d["angular"])
            self.pub_cmd.publish(msg)
        elif k == "CLASSIFY":
            self.call_classify(d.get("mode", "SIGN"))
        elif k == "SAVE_MAP":
            try:
                save_map(self.map_file, d["track_map"])
            except OSError as e:
                self.get_logger().error(f"매핑 저장 실패: {e}")
        elif k == "REPORT":
            line = format_error(d["error"], d.get("detail", "")) if "error" in d else format_event(d["event"], d.get("detail", ""))
            self.pub_report.publish(String(data=line))
            self.get_logger().info(line)

    def send_drive(self, command: str, skip: int):
        self.cancel_drive()
        if not self.drive.server_is_ready():
            self.get_logger().error("drive_node 없음")
            self.feed_later("DRIVE_DONE", result="E_LINE")
            return
        self.goal_seq += 1
        seq = self.goal_seq
        future = self.drive.send_goal_async(Drive.Goal(command=command, skip_stop_lines=int(skip)),
                                            feedback_callback=lambda fb: self.feed_later("PHASE", phase=fb.feedback.phase)
                                            if seq == self.goal_seq else None)
        future.add_done_callback(lambda f: self.on_goal_accepted(f, seq))

    def on_goal_accepted(self, future, seq):
        handle = future.result()
        if seq != self.goal_seq:                             # 그사이 다른 목표로 바뀜 → 이 목표는 취소
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.feed_later("DRIVE_DONE", result="E_LINE")
            return
        self.goal = handle
        handle.get_result_async().add_done_callback(lambda f: self.on_drive_result(f, seq))

    def on_drive_result(self, future, seq):
        if seq != self.goal_seq:
            return
        self.goal = None
        self.feed_later("DRIVE_DONE", result=future.result().result.result)

    def cancel_drive(self):
        self.goal_seq += 1                                   # 이전 목표의 결과·피드백은 버림
        if self.goal is not None:
            self.goal.cancel_goal_async()
            self.goal = None
        self.pub_cmd.publish(Twist())

    def call_classify(self, mode: str):
        if not self.classify.service_is_ready():
            self.feed_later("SIGN", label="", message="classifier_node 없음")
            return
        future = self.classify.call_async(Classify.Request(mode=mode))
        future.add_done_callback(lambda f: self.feed_later("SIGN", label=f.result().label, score=float(f.result().score),
                                                           message=f.result().message))

    def feed_later(self, kind: str, **data):
        """콜백 안(잠금 중)에서 부를 수 있게 따로 스레드로 넘긴다."""
        threading.Thread(target=self.feed, args=(kind,), kwargs=data, daemon=True).start()

    # ── 상태 보고 ─────────────────────────────────────────

    def publish_status(self, force: bool = False):
        with self.lock:
            s = self.sm.status()
            key = tuple((k, str(v)) for k, v in s.items())
            if not force and key == self.last_status:
                return
            self.last_status = key
            self.pub_status.publish(Status(
                state=s["state"], mode=s["mode"], done=s["done"], total=s["total"], cargo=s["cargo"], load=s["load"],
                target=s["target"], map=[f"{t}={s['map'].get(t) or '-'}" for t in ("WORK", "DROP_A", "DROP_B", "PARK")],
                message=s["message"]))


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_drive()
        node.destroy_node()
        rclpy.try_shutdown()

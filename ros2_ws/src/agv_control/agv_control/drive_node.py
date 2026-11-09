"""drive_node — 액션 /drive(agv_interfaces/Drive) → DriveController → /cmd_vel 20Hz (F4·F5).

    ros2 action send_goal /drive agv_interfaces/action/Drive "{command: FOLLOW_TO_EVENT}" --feedback

/line/state를 받아 라인을 따라가고, 이벤트(정지선·코너·라인 끝)·라인 끊김·비상 정지·취소에서 멈춘다.
/drive/speed(Float32, 노트북 앱 DEV SPEED)로 기본 속도, /drive/tune(String "TURN 0.6"·"CENTER -0.1", DEV TUNE)으로
회전 세기·라인 기준 위치를 주행 중에도 바꾼다. /drive/target(String "red"·"NONE", DEV TARGET)이 정해지면
/marker/state를 보고 목적지 색 스티커 쪽으로 돈다 (NONE이면 마커를 보지 않음).
동작 중이 아닐 때는 /cmd_vel을 보내지 않는다 (노트북 앱 수동 조종과 겹치지 않게).
"""
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, Float32, String

from agv_control.drive import DriveConfig, DriveController, LineInput, MarkerInput, merge_stop_line
from agv_interfaces.action import Drive
from agv_interfaces.msg import LineState, MarkerState, StopLineState
from agv_protocol.types import DriveCommand, LineEvent


STOP_LINE_FRESH_S = 0.3         # 이보다 오래된 /stop_line/state는 무시


def _event(name: str) -> LineEvent:
    try:
        return LineEvent(name)
    except ValueError:
        return LineEvent.NONE


class DriveNode(Node):
    def __init__(self):
        super().__init__("drive_node")
        p = self.declare_parameter
        d = DriveConfig()
        cfg = DriveConfig(**{k: p(k, getattr(d, k)).value for k in d.__dataclass_fields__})
        self.rate = p("rate_hz", 20.0).value
        self.ctrl = DriveController(cfg)
        self.lock = threading.Lock()
        self.line: LineInput | None = None          # 마지막 주기 이후 새로 받은 라인 상태
        self.offset = 0.0
        self.out = None                             # 진행 중 동작의 최근 출력
        self.finished = threading.Event()
        self.result = ""
        self.estop = False
        self.stop_line = (False, False, 0.0)        # 마지막 정지선 판정 (있음, 확정, 받은 시각)
        self.markers: list[MarkerInput] | None = None   # 마지막 주기 이후 새로 받은 마커 상태
        self.current_goal = None                    # 지금 실행 중인 /drive 목표 (취소 요청 대조)

        group = ReentrantCallbackGroup()
        self.pub = self.create_publisher(Twist, "/cmd_vel", QoSProfile(depth=1))
        self.create_subscription(LineState, "/line/state", self.on_line,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=group)
        self.create_subscription(StopLineState, "/stop_line/state", self.on_stop_line,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=group)
        self.create_subscription(MarkerState, "/marker/state", self.on_marker,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=group)
        self.create_subscription(String, "/drive/target", self.on_target, QoSProfile(depth=10), callback_group=group)
        self.create_subscription(Range, "/ultrasonic/range", self.on_range, qos_profile_sensor_data, callback_group=group)
        self.create_subscription(Bool, "/safety/estop", self.on_estop, QoSProfile(depth=10), callback_group=group)
        self.create_subscription(Float32, "/drive/speed", self.on_speed, QoSProfile(depth=10), callback_group=group)
        self.create_subscription(String, "/drive/tune", self.on_tune, QoSProfile(depth=10), callback_group=group)
        self.create_timer(1.0 / self.rate, self.on_timer, callback_group=group)
        self.server = ActionServer(self, Drive, "/drive", execute_callback=self.execute,
                                   goal_callback=self.on_goal, cancel_callback=self.on_cancel,
                                   callback_group=group)
        self.get_logger().info(f"drive_node 시작 (speed={cfg.speed}, kp={cfg.kp}, kd={cfg.kd})")

    # ── 입력 ─────────────────────────────────────────────

    def on_line(self, msg: LineState):
        with self.lock:
            line = LineInput(float(msg.offset), bool(msg.line_found), _event(msg.event), _event(msg.raw_event),
                             float(msg.angle), bool(msg.angle_valid), bool(msg.branch_left), bool(msg.branch_right))
            detected, confirmed, when = self.stop_line
            if time.monotonic() - when <= STOP_LINE_FRESH_S:     # 오래된 정지선 판정은 쓰지 않음
                line = merge_stop_line(line, detected, confirmed)
            self.line = line
            self.offset = float(msg.offset)

    def on_stop_line(self, msg: StopLineState):
        with self.lock:
            self.stop_line = (bool(msg.detected), bool(msg.confirmed), time.monotonic())

    def on_marker(self, msg: MarkerState):
        with self.lock:
            self.markers = [MarkerInput(c, s, float(y)) for c, s, y in zip(msg.colors, msg.sides, msg.y)]

    def on_range(self, msg: Range):
        """초음파 — 라인 주행 중 obstacle_distance 이하가 연속이면 정지 (결과 OBSTACLE)."""
        r = msg.range
        with self.lock:
            self.ctrl.set_range(r if r == r and r != float("inf") else None)

    def on_target(self, msg: String):
        """목적지 색 — 지금은 노트북 앱 (DEV TARGET), 나중에 mission_node."""
        color = None if msg.data == "NONE" else msg.data
        with self.lock:
            self.ctrl.set_target(color)
        self.get_logger().info(f"목적지 색 {msg.data}")

    def on_estop(self, msg: Bool):
        with self.lock:
            self.estop = bool(msg.data)
            if self.estop and self.ctrl.active:
                self.ctrl.cancel()
                self._finish("E_ESTOP")

    def on_speed(self, msg: Float32):
        """노트북 앱 개발자 탭 "주행 속도" (DEV SPEED) — 주행 중에도 바로 적용."""
        value = max(0.2, min(1.0, float(msg.data)))
        with self.lock:
            self.ctrl.set_speed(value)
        self.get_logger().info(f"주행 속도 {value:.2f}")

    def on_tune(self, msg: String):
        """노트북 앱 "회전 세기"·"라인 기준"·"회전 전 전진"·"제자리 회전" (DEV TUNE) — "TURN 0.6", "CENTER -0.1"."""
        try:
            key, value = msg.data.split()
            with self.lock:
                self.ctrl.set_tune(key, float(value))
        except ValueError as e:
            self.get_logger().warn(f"/drive/tune 형식 오류 {msg.data!r}: {e}")
            return
        self.get_logger().info(f"조정 {key} = {float(value):.2f}")

    def on_goal(self, goal):
        try:
            DriveCommand(goal.command)
        except ValueError:
            self.get_logger().warn(f"모르는 명령: {goal.command}")
            return GoalResponse.REJECT
        if self.estop:
            self.get_logger().warn("비상 정지 중 — 거부")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    # ── 주기 ─────────────────────────────────────────────

    def on_timer(self):
        with self.lock:
            if not self.ctrl.active:
                return
            line, self.line = self.line, None
            markers, self.markers = self.markers, None
            decision_no, note_no = self.ctrl.decision_no, self.ctrl.note_no
            out = self.ctrl.step(line, 1.0 / self.rate, markers)
            if self.ctrl.decision_no != decision_no:
                self.get_logger().info(f"마커 판단: {self.ctrl.decision}")
            if self.ctrl.note_no != note_no:
                self.get_logger().info(f"정지선: {self.ctrl.note}")
            self.out = out
            self._publish(out.linear, out.angular)
            if out.done:
                self._finish(out.result)

    def _publish(self, linear, angular):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        self.pub.publish(msg)

    def _finish(self, result: str):
        self._publish(0.0, 0.0)
        self.result = result
        self.finished.set()

    # ── 액션 ─────────────────────────────────────────────

    def on_cancel(self, goal_handle):
        """취소 요청 즉시 정지 (2026-11-06 교수 점검 2.7: 0.2초 확인 주기 동안 타이머가 접근 속도를 계속 내보내
        표지판 앞 정지 위치가 가까워졌음). 지금 실행 중인 목표의 취소일 때만 — 이전 목표의 늦은 취소로 새 목표를 멈추지 않게."""
        with self.lock:
            if goal_handle is self.current_goal and self.ctrl.active:
                self.ctrl.cancel()
                self._finish("STOPPED")
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        command = goal_handle.request.command
        with self.lock:
            self.current_goal = goal_handle
            if self.ctrl.active:                      # 이전 동작은 새 명령으로 대체
                self.ctrl.cancel()
                self._finish("STOPPED")
            self.finished.clear()
            self.result = ""
            self.line = self.markers = None
            skip = int(goal_handle.request.skip_stop_lines)
            self.ctrl.start(DriveCommand(command), skip)
        self.get_logger().info(f"시작: {command}" + (f" (정지선 {skip}개 통과)" if skip else ""))
        feedback = Drive.Feedback()
        while not self.finished.wait(0.2):
            if goal_handle.is_cancel_requested:
                break
            with self.lock:
                if self.out is not None:
                    feedback.linear, feedback.angular = float(self.out.linear), float(self.out.angular)
                feedback.offset = self.offset
                feedback.phase = self.ctrl.phase
            goal_handle.publish_feedback(feedback)
        if self.result == "STOPPED" and not goal_handle.is_cancel_requested:
            for _ in range(10):                       # on_cancel이 멈춘 직후 — 취소 상태로 바뀔 때까지 잠깐 기다림
                time.sleep(0.02)
                if goal_handle.is_cancel_requested:
                    break
        if goal_handle.is_cancel_requested:
            with self.lock:
                if goal_handle is self.current_goal and self.ctrl.active:   # on_cancel이 이미 멈췄으면 그대로
                    self.ctrl.cancel()
                    self._finish("STOPPED")
            goal_handle.canceled()
            self.get_logger().info("취소됨 → 정지")
            return Drive.Result(success=False, result="STOPPED")
        result = self.result
        ok = not result.startswith("E_") and (result != "STOPPED" or command == DriveCommand.STOP.value)
        (goal_handle.succeed if ok else goal_handle.abort)()
        self.get_logger().info(f"끝: {command} → {result}")
        return Drive.Result(success=ok, result=result)


def main(args=None):
    rclpy.init(args=args)
    node = DriveNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._publish(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()

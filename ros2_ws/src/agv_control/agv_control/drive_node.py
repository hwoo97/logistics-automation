"""drive_node — 액션 /drive(agv_interfaces/Drive) → DriveController → /cmd_vel 20Hz (F4·F5).

    ros2 action send_goal /drive agv_interfaces/action/Drive "{command: FOLLOW_TO_EVENT}" --feedback

/line/state를 받아 라인을 따라가고, 이벤트(정지선·코너·라인 끝)·라인 끊김·비상 정지·취소에서 멈춘다.
/drive/speed(Float32, 노트북 앱 DEV SPEED)로 기본 속도를 주행 중에도 바꾼다.
동작 중이 아닐 때는 /cmd_vel을 보내지 않는다 (노트북 앱 수동 조종과 겹치지 않게).
"""
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32

from agv_control.drive import DriveConfig, DriveController, LineInput
from agv_interfaces.action import Drive
from agv_interfaces.msg import LineState
from agv_protocol.types import DriveCommand, LineEvent


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

        group = ReentrantCallbackGroup()
        self.pub = self.create_publisher(Twist, "/cmd_vel", QoSProfile(depth=1))
        self.create_subscription(LineState, "/line/state", self.on_line,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=group)
        self.create_subscription(Bool, "/safety/estop", self.on_estop, QoSProfile(depth=10), callback_group=group)
        self.create_subscription(Float32, "/drive/speed", self.on_speed, QoSProfile(depth=10), callback_group=group)
        self.create_timer(1.0 / self.rate, self.on_timer, callback_group=group)
        self.server = ActionServer(self, Drive, "/drive", execute_callback=self.execute,
                                   goal_callback=self.on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=group)
        self.get_logger().info(f"drive_node 시작 (speed={cfg.speed}, kp={cfg.kp}, kd={cfg.kd})")

    # ── 입력 ─────────────────────────────────────────────

    def on_line(self, msg: LineState):
        with self.lock:
            self.line = LineInput(float(msg.offset), bool(msg.line_found), _event(msg.event), _event(msg.raw_event),
                                  float(msg.angle), bool(msg.angle_valid))
            self.offset = float(msg.offset)

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
            out = self.ctrl.step(line, 1.0 / self.rate)
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

    def execute(self, goal_handle):
        command = goal_handle.request.command
        with self.lock:
            if self.ctrl.active:                      # 이전 동작은 새 명령으로 대체
                self.ctrl.cancel()
                self._finish("STOPPED")
            self.finished.clear()
            self.result = ""
            self.line = None
            self.ctrl.start(DriveCommand(command))
        self.get_logger().info(f"시작: {command}")
        feedback = Drive.Feedback()
        while not self.finished.wait(0.2):
            if goal_handle.is_cancel_requested:
                with self.lock:
                    self.ctrl.cancel()
                    self._finish("STOPPED")
                goal_handle.canceled()
                self.get_logger().info("취소됨 → 정지")
                return Drive.Result(success=False, result="STOPPED")
            with self.lock:
                if self.out is not None:
                    feedback.linear, feedback.angular = float(self.out.linear), float(self.out.angular)
                feedback.offset = self.offset
            goal_handle.publish_feedback(feedback)
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

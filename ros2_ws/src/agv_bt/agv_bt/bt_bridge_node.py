"""bt_bridge_node — 노트북 앱 ↔ ROS 토픽 (F10·F11, 블루투스 규약 3-1절 DEV).

판단은 BridgeCore, 통신은 RfcommServer, 영상 JPEG는 VideoStreamer가 맡고 이 노드는 토픽 연결만 한다.
"""
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

from agv_bt.bridge_core import BridgeCore
from agv_bt.rfcomm_server import RfcommServer
from agv_bt.sysinfo import SysMonitor
from agv_bt.video_streamer import VideoStreamer
from agv_protocol.messages import LINE_TUNE_KEYS, format_dev

WATCH_PERIOD_S = 0.2            # RANGE·LINE·MARKER 5Hz


class BtBridgeNode(Node):
    def __init__(self):
        super().__init__("bt_bridge_node")
        p = self.declare_parameter
        transport = p("transport", "rfcomm").value            # rfcomm / tcp (PC·Pi 시험용)
        tcp_port = p("tcp_port", 9000).value
        standalone = p("standalone", True).value
        fps = p("video_fps", 5.0).value
        size = p("video_size", [320, 240]).value
        quality = p("jpeg_quality", 50).value
        camera_topic = p("camera_topic", "/camera/image_raw").value

        self.lock = threading.Lock()                          # 서버 스레드·ROS 콜백이 core를 함께 씀
        self.core = BridgeCore(self, standalone=standalone)
        self.streamer = VideoStreamer(fps, size, quality)
        self.sys = SysMonitor()
        self._last_range_sent = 0.0
        self._last_line_sent = 0.0
        self._last_marker_sent = 0.0
        self._stop_line = (False, False, 0.0)                 # 마지막 정지선 판정 (있음, 확정, 받은 시각)

        q10 = QoSProfile(depth=10)
        self.pub_estop = self.create_publisher(Bool, "/safety/estop", q10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", QoSProfile(depth=1))
        self.pub_led = self.create_publisher(String, "/led/state", q10)
        self.pub_pose = self.create_publisher(String, "/pantilt/pose", q10)
        self.pub_angle = self.create_publisher(Float32MultiArray, "/dev/pantilt_angle", q10)
        self.pub_wheels = self.create_publisher(Float32MultiArray, "/dev/wheels", QoSProfile(depth=1))
        self.pub_speed = self.create_publisher(Float32, "/drive/speed", q10)
        self.pub_tune = self.create_publisher(String, "/drive/tune", q10)
        self.pub_line_tune = self.create_publisher(String, "/line/tune", q10)
        self.pub_target = self.create_publisher(String, "/drive/target", q10)
        self.create_subscription(Range, "/ultrasonic/range", self.on_range, qos_profile_sensor_data)
        self.create_subscription(Image, camera_topic, self.on_image, qos_profile_sensor_data)
        try:                                                  # agv_interfaces가 없으면 LINE만 빠짐
            from agv_interfaces.msg import LineState, MarkerState, StopLineState
            best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(LineState, "/line/state", self.on_line_state, best_effort)
            self.create_subscription(StopLineState, "/stop_line/state", self.on_stop_line, best_effort)
            self.create_subscription(MarkerState, "/marker/state", self.on_marker_state, best_effort)
        except ImportError:
            self.get_logger().warn("agv_interfaces 없음 — DEV LINE 보내지 않음")
        self.drive_client, self._drive_goal = None, None    # 노트북 앱 "라인 주행 시작/정지" → /drive 액션
        self._drive_phase = "FOLLOW"
        try:
            from rclpy.action import ActionClient
            from agv_interfaces.action import Drive
            self.Drive = Drive
            self.drive_client = ActionClient(self, Drive, "/drive")
        except ImportError:
            self.get_logger().warn("agv_interfaces 없음 — DEV DRIVE 사용 불가")

        self.server = RfcommServer(on_line=self.on_line, on_connect=self.on_connect,
                                   transport=transport, tcp_port=tcp_port)
        self.server.start()
        self.create_timer(0.1, self.on_heartbeat_timer)
        self.create_timer(0.5, self.on_status_timer)
        self.create_timer(1.0, self.on_sys_timer)
        self.create_timer(0.05, self.on_video_timer)
        self.get_logger().info(f"bt_bridge_node 대기: {self.server.where} (standalone={standalone})")

    # ── BridgeCore 출력 (io) ──────────────────────────────

    def send(self, line):
        self.server.send_line(line)

    def estop(self, on):
        self.pub_estop.publish(Bool(data=on))
        self.get_logger().warn("비상 정지" if on else "비상 정지 해제")

    def cmd_vel(self, v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.pub_cmd.publish(msg)

    def led(self, state):
        self.pub_led.publish(String(data=state))

    def pose(self, name):
        self.pub_pose.publish(String(data=name))

    def servo(self, pan, tilt):
        self.pub_angle.publish(Float32MultiArray(data=[float(pan), float(tilt)]))

    def wheels(self, values):
        self.pub_wheels.publish(Float32MultiArray(data=[float(v) for v in values]))

    def speed(self, value):
        self.pub_speed.publish(Float32(data=float(value)))

    def tune(self, key, value):
        """DEV TUNE — 프레임 수는 line_node(/line/tune), 회전 세기·라인 기준은 drive_node(/drive/tune)."""
        if key in LINE_TUNE_KEYS:
            self.pub_line_tune.publish(String(data=f"{key} {int(value)}"))
        else:
            self.pub_tune.publish(String(data=f"{key} {float(value)}"))

    def target(self, color):
        """DEV TARGET — 시험용 목적지 색. drive_node는 소문자 색 이름 또는 NONE을 받는다."""
        self.pub_target.publish(String(data="NONE" if color == "NONE" else color.lower()))

    def drive(self, action):
        """DEV DRIVE FOLLOW|STOP — drive_node에 FOLLOW_TO_EVENT 목표를 보내거나 취소. 결과는 DEV DRIVE로 알림."""
        if self.drive_client is None:
            self.send(format_dev("DRIVE", "REJECTED", "agv_interfaces 없음"))
            return
        if action == "STOP":
            if self._drive_goal is not None:
                self._drive_goal.cancel_goal_async()            # drive_node가 정지 후 STOPPED로 끝냄
            else:
                self.send(format_dev("DRIVE", "DONE", "STOPPED"))
            return
        if not self.drive_client.server_is_ready():
            self.send(format_dev("DRIVE", "REJECTED", "drive_node 없음"))
            return
        self.core.set_driving(True)                             # 응답 전에도 겹치는 명령을 막음
        self._drive_phase = "FOLLOW"
        future = self.drive_client.send_goal_async(self.Drive.Goal(command="FOLLOW_TO_EVENT"),
                                                   feedback_callback=self._on_drive_feedback)
        future.add_done_callback(self._on_drive_accepted)

    def _on_drive_feedback(self, msg):
        """단계가 바뀔 때만 앱에 알림 — 라인 끝 재확인(RECHECK 2/3), 정지선·코너 확인(VERIFY STOP_LINE),
        마커 확인(MARKER red), 회전(TURN LEFT), 다시 출발(RUNNING)."""
        phase = msg.feedback.phase
        if phase == self._drive_phase:
            return
        self._drive_phase = phase
        if phase.startswith(("RECHECK", "VERIFY", "MARKER", "TURN")):   # "RECHECK 2/3", "VERIFY STOP_LINE"
            self.send(format_dev("DRIVE", *phase.split()))
        else:
            self.send(format_dev("DRIVE", "RUNNING"))

    def _on_drive_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.core.set_driving(False)                        # 값 하나만 바꿈 — 잠금 없이 (콜백이 잠금 안에서 불릴 수 있음)
            self.send(format_dev("DRIVE", "REJECTED", "drive_node 거부 (비상 정지 중?)"))
            return
        self._drive_goal = handle
        self.send(format_dev("DRIVE", "RUNNING"))
        handle.get_result_async().add_done_callback(self._on_drive_result)

    def _on_drive_result(self, future):
        result = future.result().result
        self._drive_goal = None
        self.core.set_driving(False)
        self.send(format_dev("DRIVE", "DONE", result.result or "STOPPED"))
        self.get_logger().info(f"라인 주행 끝: {result.result}")

    def node_names(self):
        return self.get_node_names()

    def mission(self, cmd):
        self.get_logger().error(f"mission_node 연동 전 (standalone=False 미지원): {cmd.line()}")

    # ── 통신 콜백 (서버 스레드) ────────────────────────────

    def on_line(self, line):
        if line != "PING":
            self.get_logger().info(f"수신 {line}")
        with self.lock:
            self.core.on_line(line)

    def on_connect(self):
        self.get_logger().info("노트북 연결됨")
        with self.lock:
            self.core.on_connect()

    # ── ROS 콜백·타이머 ───────────────────────────────────

    def on_heartbeat_timer(self):
        with self.lock:
            self.core.on_heartbeat(self.server.ever_connected, self.server.is_alive())

    def on_status_timer(self):
        if self.server.connected:
            self.send(self.core.status_line())

    def on_sys_timer(self):
        if "SYS" in self.core.watches and self.server.connected:
            temp, cpu, mem = self.sys.sample()
            self.send(format_dev("SYS", round(temp, 1), round(cpu, 0), round(mem, 0)))

    def on_range(self, msg: Range):
        now = time.monotonic()
        if "RANGE" in self.core.watches and now - self._last_range_sent >= WATCH_PERIOD_S:
            self._last_range_sent = now
            if msg.range == msg.range and msg.range != float("inf"):    # NaN·inf 제외
                self.send(format_dev("RANGE", round(msg.range, 3)))

    def on_line_state(self, msg):
        now = time.monotonic()
        if "LINE" in self.core.watches and now - self._last_line_sent >= WATCH_PERIOD_S:
            self._last_line_sent = now
            angle = round(msg.angle, 1) if msg.angle_valid else "-"
            event, raw = msg.event or "NONE", msg.raw_event or "NONE"
            detected, confirmed, when = self._stop_line
            if now - when <= 0.3:                             # 앱에는 한 줄로 — 정지선 판정을 합쳐서 보냄
                event = "STOP_LINE" if confirmed else event
                raw = "STOP_LINE" if detected or confirmed else raw
            self.send(format_dev("LINE", round(msg.offset, 2), event, raw, angle))

    def on_stop_line(self, msg):
        self._stop_line = (bool(msg.detected), bool(msg.confirmed), time.monotonic())

    def on_marker_state(self, msg):
        now = time.monotonic()
        if "MARKER" in self.core.watches and now - self._last_marker_sent >= WATCH_PERIOD_S:
            self._last_marker_sent = now
            self.send(format_dev("MARKER", *(f"{c}:{s}:{y:.0f}" for c, s, y in zip(msg.colors, msg.sides, msg.y))))

    def on_image(self, msg: Image):
        self.streamer.update(msg.encoding, msg.height, msg.width, msg.step, msg.data)

    def on_video_timer(self):
        if self.server._video is None:
            return
        try:
            jpeg = self.streamer.next_frame(time.monotonic())
        except ValueError as e:
            self.get_logger().warn(f"영상 변환 실패: {e}", throttle_duration_sec=10.0)
            return
        if jpeg:
            self.server.send_frame(jpeg)

    def destroy_node(self):
        self.server.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BtBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

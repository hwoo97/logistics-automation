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
from std_msgs.msg import Bool, Float32MultiArray, String

from agv_bt.bridge_core import BridgeCore
from agv_bt.rfcomm_server import RfcommServer
from agv_bt.sysinfo import SysMonitor
from agv_bt.video_streamer import VideoStreamer
from agv_protocol.messages import format_dev

WATCH_PERIOD_S = 0.2            # RANGE·LINE 5Hz


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

        q10 = QoSProfile(depth=10)
        self.pub_estop = self.create_publisher(Bool, "/safety/estop", q10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", QoSProfile(depth=1))
        self.pub_led = self.create_publisher(String, "/led/state", q10)
        self.pub_pose = self.create_publisher(String, "/pantilt/pose", q10)
        self.pub_angle = self.create_publisher(Float32MultiArray, "/dev/pantilt_angle", q10)
        self.pub_wheels = self.create_publisher(Float32MultiArray, "/dev/wheels", QoSProfile(depth=1))
        self.create_subscription(Range, "/ultrasonic/range", self.on_range, qos_profile_sensor_data)
        self.create_subscription(Image, camera_topic, self.on_image, qos_profile_sensor_data)
        try:                                                  # agv_interfaces가 없으면 LINE만 빠짐
            from agv_interfaces.msg import LineState
            self.create_subscription(LineState, "/line/state", self.on_line_state,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        except ImportError:
            self.get_logger().warn("agv_interfaces 없음 — DEV LINE 보내지 않음")

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
            self.send(format_dev("LINE", round(msg.offset, 2), msg.event or "NONE", msg.raw_event or "NONE"))

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

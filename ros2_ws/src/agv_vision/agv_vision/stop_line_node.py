"""stop_line_node — /camera/image_raw + /line/state(오프셋) → StopLineDetector → /stop_line/state (F4).
line_node와 분리된 정지선 판정. /line/tune "CONFIRM n"(노트북 앱 "확인 프레임")으로 확정 프레임 수를 바꾼다."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from agv_bt.video_streamer import to_bgr
from agv_interfaces.msg import LineState, StopLineState
from agv_vision.stop_line import StopLineConfig, StopLineDetector


class StopLineNode(Node):
    def __init__(self):
        super().__init__("stop_line_node")
        p = self.declare_parameter
        d = StopLineConfig()
        cfg = StopLineConfig(
            roi_ratio=p("roi_ratio", d.roi_ratio).value,
            black_ratio=p("black_ratio", d.black_ratio).value,
            black_v_max=p("black_v_max", d.black_v_max).value,
            black_s_max=p("black_s_max", d.black_s_max).value,
            black_blur=p("black_blur", d.black_blur).value,
            default_line_px=p("default_line_px", d.default_line_px).value,
            max_line_px=p("max_line_px", d.max_line_px).value,
            width_ratio=tuple(p("width_ratio", list(d.width_ratio)).value),
            min_thickness=p("min_thickness", d.min_thickness).value,
            side_min_ratio=p("side_min_ratio", d.side_min_ratio).value,
            gap_cols=p("gap_cols", d.gap_cols).value,
            max_tilt_deg=p("max_tilt_deg", d.max_tilt_deg).value,
            edge_ratio=p("edge_ratio", d.edge_ratio).value,
            confirm_frames=p("confirm_frames", d.confirm_frames).value,
        )
        self.detector = StopLineDetector(cfg)
        self.offset = 0.0                         # 마지막 라인 오프셋 (라인 노드와 프레임이 조금 어긋나도 판정엔 지장 없음)
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(StopLineState, "/stop_line/state", best_effort)
        self.create_subscription(LineState, "/line/state", self.on_line, best_effort)
        self.create_subscription(Image, p("camera_topic", "/camera/image_raw").value, self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(String, "/line/tune", self.on_tune, 10)
        self.get_logger().info(f"stop_line_node 시작 (두께 {cfg.min_thickness}, 폭 {cfg.width_ratio})")

    def on_line(self, msg: LineState):
        if msg.line_found:
            self.offset = float(msg.offset)

    def on_tune(self, msg: String):
        """노트북 앱 "확인 프레임" (DEV TUNE CONFIRM). 다른 항목은 line_node 몫."""
        parts = msg.data.split()
        if len(parts) == 2 and parts[0] == "CONFIRM":
            self.detector.set_frames(int(float(parts[1])))
            self.get_logger().info(f"조정 CONFIRM = {parts[1]}프레임")

    def on_image(self, msg: Image):
        try:
            bgr = to_bgr(msg.encoding, msg.height, msg.width, msg.step, msg.data)
        except ValueError as e:
            self.get_logger().warn(f"영상 변환 실패: {e}", throttle_duration_sec=10.0)
            return
        r = self.detector.process(bgr, self.offset)
        self.pub.publish(StopLineState(detected=r.detected, confirmed=r.confirmed, angle=float(r.angle)))


def main(args=None):
    rclpy.init(args=args)
    node = StopLineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

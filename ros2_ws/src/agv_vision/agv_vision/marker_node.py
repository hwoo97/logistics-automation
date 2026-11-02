"""marker_node — /camera/image_raw + /line/state(오프셋·각도) → MarkerDetector → /marker/state (F6).
보이는 스티커의 색·좌우·화면 높이만 보낸다. 목적지와 비교해 움직임을 정하는 것은 drive_node."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image

from agv_bt.video_streamer import to_bgr
from agv_interfaces.msg import LineState, MarkerState
from agv_vision.marker import MarkerConfig, MarkerDetector, config_fields


def _param_value(value):
    return list(value) if isinstance(value, tuple) else value


class MarkerNode(Node):
    def __init__(self):
        super().__init__("marker_node")
        d = MarkerConfig()
        values = {k: self.declare_parameter(k, _param_value(getattr(d, k))).value for k in config_fields()}
        cfg = MarkerConfig(**{k: tuple(v) if isinstance(getattr(d, k), tuple) else v for k, v in values.items()})
        self.detector = MarkerDetector(cfg)
        self.offset, self.angle = 0.0, None       # 마지막 라인 위치 (라인 노드와 프레임이 조금 어긋나도 좌우 판단엔 지장 없음)
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(MarkerState, "/marker/state", best_effort)
        self.create_subscription(LineState, "/line/state", self.on_line, best_effort)
        self.create_subscription(Image, self.declare_parameter("camera_topic", "/camera/image_raw").value,
                                 self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f"marker_node 시작 (면적 ≥ {cfg.min_area}, 빨강 {cfg.red})")

    def on_line(self, msg: LineState):
        if msg.line_found:
            self.offset = float(msg.offset)
            self.angle = float(msg.angle) if msg.angle_valid else None

    def on_image(self, msg: Image):
        try:
            bgr = to_bgr(msg.encoding, msg.height, msg.width, msg.step, msg.data)
        except ValueError as e:
            self.get_logger().warn(f"영상 변환 실패: {e}", throttle_duration_sec=10.0)
            return
        found = self.detector.process(bgr, self.offset, self.angle)
        self.pub.publish(MarkerState(colors=[m.color for m in found], sides=[m.side for m in found],
                                     y=[m.y for m in found]))


def main(args=None):
    rclpy.init(args=args)
    node = MarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

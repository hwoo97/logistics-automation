"""line_node — /camera/image_raw(최신 1장) → LineDetector → /line/state (F4). 파라미터 enabled로 끄고 켬.
/line/tune(String "CONFIRM 3"·"LINE_END 10", 노트북 앱 DEV TUNE)으로 확정 프레임 수를 실행 중에 바꾼다."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from agv_bt.video_streamer import to_bgr
from agv_interfaces.msg import LineState
from agv_vision.line_detect import LineConfig, LineDetector


class LineNode(Node):
    def __init__(self):
        super().__init__("line_node")
        p = self.declare_parameter
        d = LineConfig()
        cfg = LineConfig(
            roi_ratio=p("roi_ratio", d.roi_ratio).value,
            black_v_range=tuple(p("black_v_range", list(d.black_v_range)).value),
            black_s_max=p("black_s_max", d.black_s_max).value,
            min_pixels=p("min_pixels", d.min_pixels).value,
            column_fill=p("column_fill", d.column_fill).value,
            edge_ratio=p("edge_ratio", d.edge_ratio).value,
            stop_line_width_ratio=tuple(p("stop_line_width_ratio", list(d.stop_line_width_ratio)).value),
            stop_min_thickness=p("stop_min_thickness", d.stop_min_thickness).value,
            ahead_center_ratio=p("ahead_center_ratio", d.ahead_center_ratio).value,
            default_line_px=p("default_line_px", d.default_line_px).value,
            max_line_px=p("max_line_px", d.max_line_px).value,
            angle_min_rows=p("angle_min_rows", d.angle_min_rows).value,
            center_weight_min=p("center_weight_min", d.center_weight_min).value,
            prev_weight_min=p("prev_weight_min", d.prev_weight_min).value,
            bottom_bonus=p("bottom_bonus", d.bottom_bonus).value,
            confirm_frames=p("confirm_frames", d.confirm_frames).value,
            line_end_frames=p("line_end_frames", d.line_end_frames).value,
        )
        self.enabled = p("enabled", True).value
        self.detector = LineDetector(cfg)
        self.pub = self.create_publisher(LineState, "/line/state",
                                         QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Image, p("camera_topic", "/camera/image_raw").value, self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(String, "/line/tune", self.on_tune, 10)
        self.add_on_set_parameters_callback(self.on_params)
        self.get_logger().info(f"line_node 시작 (enabled={self.enabled}, roi={cfg.roi_ratio})")

    def on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for prm in params:
            if prm.name == "enabled":
                self.enabled = bool(prm.value)
        return SetParametersResult(successful=True)

    def on_tune(self, msg: String):
        """노트북 앱 "확인 프레임"·"라인 끝 프레임" (DEV TUNE) — "CONFIRM 3", "LINE_END 10"."""
        try:
            key, value = msg.data.split()
            self.detector.set_frames(key, int(float(value)))
        except ValueError as e:
            self.get_logger().warn(f"/line/tune 형식 오류 {msg.data!r}: {e}")
            return
        self.get_logger().info(f"조정 {key} = {int(float(value))}프레임")

    def on_image(self, msg: Image):
        if not self.enabled:
            return
        try:
            bgr = to_bgr(msg.encoding, msg.height, msg.width, msg.step, msg.data)
        except ValueError as e:
            self.get_logger().warn(f"영상 변환 실패: {e}", throttle_duration_sec=10.0)
            return
        r = self.detector.process(bgr)
        self.pub.publish(LineState(offset=float(r.offset), line_found=r.line_found,
                                   event=r.event.value, raw_event=r.raw_event.value,
                                   angle=float(r.angle), angle_valid=r.angle_valid))


def main(args=None):
    rclpy.init(args=args)
    node = LineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

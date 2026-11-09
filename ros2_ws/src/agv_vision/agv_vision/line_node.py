"""line_node — /camera/image_raw(최신 1장) → LineDetector → /line/state, StopLineDetector → /stop_line/state (F4).
영상 노드를 줄이려고 정지선 판정도 여기서 같은 프레임으로 한다 (2026-11-02). 파라미터 enabled로 끄고 켬.
/line/tune(String "CONFIRM 3"·"LINE_END 10", 노트북 앱 DEV TUNE)으로 확정 프레임 수를 실행 중에 바꾼다."""

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from agv_bt.video_streamer import to_bgr
from agv_interfaces.msg import LineState, StopLineState
from agv_vision.line_detect import LineConfig, LineDetector
from agv_vision.stop_line import MissTracker, StopLineConfig, StopLineDetector


class LineNode(Node):
    def __init__(self):
        super().__init__("line_node")
        p = self.declare_parameter
        d = LineConfig()
        cfg = LineConfig(
            roi_ratio=p("roi_ratio", d.roi_ratio).value,
            black_ratio=p("black_ratio", d.black_ratio).value,
            black_v_max=p("black_v_max", d.black_v_max).value,
            black_blur=p("black_blur", d.black_blur).value,
            black_s_max=p("black_s_max", d.black_s_max).value,
            min_pixels=p("min_pixels", d.min_pixels).value,
            column_fill=p("column_fill", d.column_fill).value,
            edge_ratio=p("edge_ratio", d.edge_ratio).value,
            corner_min_width=p("corner_min_width", d.corner_min_width).value,
            ahead_center_ratio=p("ahead_center_ratio", d.ahead_center_ratio).value,
            default_line_px=p("default_line_px", d.default_line_px).value,
            max_line_px=p("max_line_px", d.max_line_px).value,
            angle_min_rows=p("angle_min_rows", d.angle_min_rows).value,
            center_weight_min=p("center_weight_min", d.center_weight_min).value,
            prev_weight_min=p("prev_weight_min", d.prev_weight_min).value,
            bottom_bonus=p("bottom_bonus", d.bottom_bonus).value,
            confirm_frames=p("confirm_frames", d.confirm_frames).value,
            line_end_frames=p("line_end_frames", d.line_end_frames).value,
            branch_top_ratio=p("branch_top_ratio", d.branch_top_ratio).value,
            branch_reach=p("branch_reach", d.branch_reach).value,
            branch_rows=p("branch_rows", d.branch_rows).value,
            sticker_as_line=p("sticker_as_line", d.sticker_as_line).value,
            sticker_s_min=p("sticker_s_min", d.sticker_s_min).value,
            sticker_v_min=p("sticker_v_min", d.sticker_v_min).value,
        )
        self.enabled = p("enabled", True).value
        self.detector = LineDetector(cfg)
        s = StopLineConfig()
        stop_cfg = StopLineConfig(                      # 검정 추출·라인 굵기는 라인과 같은 값, 나머지는 stop_ 파라미터
            roi_ratio=p("stop_roi_ratio", s.roi_ratio).value,
            black_ratio=cfg.black_ratio,
            black_v_max=cfg.black_v_max,
            black_s_max=cfg.black_s_max,
            black_blur=cfg.black_blur,
            default_line_px=cfg.default_line_px,
            max_line_px=cfg.max_line_px,
            width_ratio=tuple(p("stop_width_ratio", list(s.width_ratio)).value),
            min_thickness=p("stop_min_thickness", s.min_thickness).value,
            side_min_ratio=p("stop_side_min_ratio", s.side_min_ratio).value,
            gap_cols=p("stop_gap_cols", s.gap_cols).value,
            max_tilt_deg=p("stop_max_tilt_deg", s.max_tilt_deg).value,
            edge_ratio=cfg.edge_ratio,
            confirm_frames=cfg.confirm_frames,
            hold_frames=p("stop_hold_frames", s.hold_frames).value,
            hold_max_thick=p("stop_hold_max_thick", s.hold_max_thick).value,
            keep_thickness=p("stop_keep_thickness", s.keep_thickness).value,
            sticker_as_line=cfg.sticker_as_line,
            sticker_s_min=cfg.sticker_s_min,
            sticker_v_min=cfg.sticker_v_min,
            junction_check=p("stop_junction_check", s.junction_check).value,
            junction_reach=p("stop_junction_reach", s.junction_reach).value,
        )
        self.stop_line = StopLineDetector(stop_cfg)
        self.offset = 0.0                               # 마지막 라인 오프셋 — 라인을 놓친 프레임의 정지선 판정에 씀
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(LineState, "/line/state", best_effort)
        self.pub_stop = self.create_publisher(StopLineState, "/stop_line/state", best_effort)
        self.create_subscription(Image, p("camera_topic", "/camera/image_raw").value, self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(String, "/line/tune", self.on_tune, 10)
        self.add_on_set_parameters_callback(self.on_params)
        self.pub_log = self.create_publisher(String, "/agv/log", QoSProfile(depth=10))   # 앱 콘솔 동작 로그 (2026-11-09)
        self.miss = MissTracker(p("stop_miss_frames", 3).value)   # 놓침 의심: 정지선 후보가 이만큼 이어졌는데 판정 0
        self.get_logger().info(f"line_node 시작 (enabled={self.enabled}, roi={cfg.roi_ratio})")

    def _check_miss(self, s):
        """정지선 놓침 의심 → 앱 콘솔 (/agv/log "STOP …"). Pi 로그(1초 진단 요약·경고)는 2026-11-09 사용자 요청으로 없앰."""
        missed = self.miss.push(s)
        if missed:
            frames, top, last = missed
            why = ", ".join(f"{k} {n}" for k, n in top)
            self.pub_log.publish(String(data=f"STOP 정지선 놓침 의심: 후보 {frames}장 이어졌는데 판정 0 — 탈락 [{why}] 예: {last}"))

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
            if key == "CONFIRM":
                self.stop_line.set_frames(int(float(value)))
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
        c = self.detector.cfg
        small = cv2.resize(bgr, (c.width, c.height), interpolation=cv2.INTER_AREA)   # 두 판정이 같은 축소 영상을 씀
        r = self.detector.process(small)
        if r.line_found:
            self.offset = float(r.offset)
        s = self.stop_line.process(small, self.offset)
        self.pub_stop.publish(StopLineState(detected=s.detected, confirmed=s.confirmed, angle=float(s.angle)))
        self._check_miss(s)
        self.pub.publish(LineState(offset=float(r.offset), line_found=r.line_found,
                                   event=r.event.value, raw_event=r.raw_event.value,
                                   angle=float(r.angle), angle_valid=r.angle_valid,
                                   branch_left=r.branch_left, branch_right=r.branch_right))


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

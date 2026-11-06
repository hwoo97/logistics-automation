"""classifier_node — 서비스 /vision/classify(SIGN) → 새 영상 5장 추론 → 확정 라벨 (F7).

모델은 Teachable Machine에서 내보낸 sign.tflite + labels_sign.txt (agv_bringup/models/). 파일이 없으면 노드는 뜨고
요청마다 "모델 없음"으로 실패를 돌려준다 (mission_node가 E_SIGN 처리). 영상은 요청이 왔을 때만 추론한다.
카메라 영상도 요청 중에만 구독한다 (2026-11-06) — 늘 받으면 주행 중 Pi 부하가 늘어 line_node 처리 프레임이 줄고
정지선 확정(3프레임)이 늦어져 정지선을 지나쳤음 (agv.launch.py 도착 시험).
"""
import os
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from agv_bt.video_streamer import to_bgr
from agv_interfaces.srv import Classify
from agv_vision.classifier import DEFAULT_LABEL_MAP, TfliteClassifier, decide, map_label, parse_label_map


def _model_dir() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("agv_bringup"), "models")
    except Exception:                                       # noqa: BLE001 — 패키지가 없으면 상대 경로
        return "models"


class ClassifierNode(Node):
    def __init__(self):
        super().__init__("classifier_node")
        p = self.declare_parameter
        models = _model_dir()
        # 파일 이름만 주면 설치 폴더 models/ 안에서 찾음 (Teachable Machine 기본 이름 model_unquant.tflite·labels.txt)
        model = os.path.join(models, p("sign_model", "model_unquant.tflite").value)
        labels = os.path.join(models, p("sign_labels", "labels.txt").value)
        self.label_map = parse_label_map(p("label_map", list(DEFAULT_LABEL_MAP)).value)
        self.frames = p("frames", 5).value                  # 추론할 새 영상 수
        self.need = p("need", 3).value                      # 같은 라벨이 이만큼
        self.min_score = p("min_score", 0.8).value
        self.timeout_s = p("timeout_s", 3.0).value
        self.classifier = None
        self.error = ""
        try:
            self.classifier = TfliteClassifier(model, labels)
            names = [f"{lb}→{map_label(lb, self.label_map)}" for lb in self.classifier.labels]
            self.get_logger().info(f"표지판 모델 {model} — 라벨 {names}")
        except Exception as e:                              # noqa: BLE001 — 모델 없음·런타임 없음 모두 노드는 살림
            self.error = f"모델 없음: {e}"
            self.get_logger().error(f"{self.error} — Teachable Machine 모델을 {models}에 넣고 다시 빌드")
        self.cond = threading.Condition()
        self.latest, self.seq = None, 0                     # 마지막 영상 (변환 전)과 번호
        self.camera_topic = p("camera_topic", "/camera/image_raw").value
        self.group = ReentrantCallbackGroup()
        self.create_service(Classify, "/vision/classify", self.on_classify, callback_group=self.group)

    def on_image(self, msg: Image):
        with self.cond:
            self.latest, self.seq = msg, self.seq + 1
            self.cond.notify_all()

    def _next_frame(self, after: int, deadline: float):
        with self.cond:
            while self.seq <= after:
                left = deadline - time.monotonic()
                if left <= 0:
                    return None, after
                self.cond.wait(left)
            return self.latest, self.seq

    def on_classify(self, req, res):
        res.label, res.score = "", 0.0
        if self.classifier is None:
            res.message = self.error
            return res
        deadline = time.monotonic() + self.timeout_s
        seq, results = self.seq, []
        sub = self.create_subscription(Image, self.camera_topic, self.on_image, qos_profile_sensor_data,
                                       callback_group=self.group)   # 요청 중에만 영상 받음
        try:
            while len(results) < self.frames:
                msg, seq = self._next_frame(seq, deadline)
                if msg is None:
                    break
                try:
                    label, score = self.classifier.predict(to_bgr(msg.encoding, msg.height, msg.width, msg.step,
                                                                  msg.data))
                    results.append((map_label(label, self.label_map), score))
                except ValueError as e:
                    res.message = f"영상 변환 실패: {e}"
                    return res
        finally:
            self.destroy_subscription(sub)
            with self.cond:
                self.latest = None
        found = decide(results, self.min_score, self.need)
        if found:
            res.label, res.score = found[0], found[1]
        else:
            res.message = f"확정 못 함 ({len(results)}장: {', '.join(f'{lb} {s:.2f}' for lb, s in results)})"
        self.get_logger().info(f"표지판 {res.label or '-'} {res.score:.2f} {res.message}")
        return res


def main(args=None):
    rclpy.init(args=args)
    node = ClassifierNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

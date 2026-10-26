"""카메라 토픽 점검 — Raspberry Pi 5에서 camera_ros 실행 중에 사용.

정해진 시간 동안 영상 토픽을 받아 해상도·인코딩·FPS·프레임 간격을 측정하고,
마지막 프레임을 JPEG로 저장한다(라인·마커가 보이는지 눈으로 확인용).

    python3 camera_topic_check.py                       # /camera/image_raw, 10초
    python3 camera_topic_check.py --seconds 20 --save captures/line.jpg
"""
import argparse
import os
import statistics
import time

import numpy as np

MIN_FPS = 15.0       # 라인 추종에 필요한 최소 프레임 (G1 통과 기준)


def to_bgr(encoding, height, width, step, data):
    """sensor_msgs/Image 데이터를 OpenCV BGR 배열로 변환 (행 끝 여백 step 처리 포함)."""
    import cv2
    raw = np.frombuffer(bytes(data), dtype=np.uint8).reshape(height, step)
    enc = encoding.lower()
    if enc in ("bgr8", "rgb8"):
        img = raw[:, : width * 3].reshape(height, width, 3)
        return img if enc == "bgr8" else cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if enc in ("bgra8", "rgba8"):
        img = raw[:, : width * 4].reshape(height, width, 4)
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR if enc == "bgra8" else cv2.COLOR_RGBA2BGR)
    if enc == "mono8":
        return cv2.cvtColor(raw[:, :width].copy(), cv2.COLOR_GRAY2BGR)
    if enc in ("yuv422_yuy2", "yuyv"):
        img = raw[:, : width * 2].reshape(height, width, 2)
        return cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUYV)
    if enc in ("uyvy", "yuv422"):
        img = raw[:, : width * 2].reshape(height, width, 2)
        return cv2.cvtColor(img, cv2.COLOR_YUV2BGR_UYVY)
    raise ValueError(f"변환하지 않는 인코딩: {encoding}")


def summarize(stamps):
    """수신 시각 목록 → (FPS, 평균 간격 ms, 최대 간격 ms)."""
    if len(stamps) < 2:
        return 0.0, 0.0, 0.0
    gaps = [(b - a) * 1000 for a, b in zip(stamps, stamps[1:])]
    fps = (len(stamps) - 1) / (stamps[-1] - stamps[0])
    return fps, statistics.mean(gaps), max(gaps)


def main():
    parser = argparse.ArgumentParser(description="카메라 토픽 FPS·해상도 점검")
    parser.add_argument("--topic", default="/camera/image_raw")
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--save", default="captures/camera_check.jpg", help="마지막 프레임 저장 경로")
    args = parser.parse_args()

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    rclpy.init()
    node = rclpy.create_node("camera_topic_check")
    stamps, last = [], {}

    def on_image(msg):
        stamps.append(time.perf_counter())
        last["msg"] = msg

    node.create_subscription(Image, args.topic, on_image, qos_profile_sensor_data)
    print(f"{args.topic} 수신 대기 ({args.seconds:.0f}초)...")
    end = time.perf_counter() + args.seconds
    while time.perf_counter() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    if not stamps:
        print("프레임 0장 — camera_node 실행 여부, 토픽 이름(ros2 topic list) 확인")
        raise SystemExit(1)

    msg = last["msg"]
    fps, gap_avg, gap_max = summarize(stamps)
    print(f"프레임 {len(stamps)}장, 해상도 {msg.width}×{msg.height}, 인코딩 {msg.encoding}, step {msg.step}")
    print(f"FPS {fps:.1f}, 프레임 간격 평균 {gap_avg:.1f}ms / 최대 {gap_max:.1f}ms")

    saved = False
    try:
        import cv2
        img = to_bgr(msg.encoding, msg.height, msg.width, msg.step, msg.data)
        os.makedirs(os.path.dirname(args.save) or ".", exist_ok=True)
        saved = cv2.imwrite(args.save, img)
        print(f"마지막 프레임 저장: {args.save}" if saved else "저장 실패")
    except (ImportError, ValueError) as e:
        print(f"프레임 저장 생략: {e}")

    print("\n[판정]")
    print(f"- FPS {MIN_FPS:.0f} 이상: {'통과' if fps >= MIN_FPS else '실패'}")
    print(f"- 최대 간격 < 200ms (끊김 없음): {'통과' if gap_max < 200 else '실패'}")
    print(f"- 사진 확인: {'저장된 사진에서 초점·색·방향 확인' if saved else '저장 안 됨'}")


if __name__ == "__main__":
    main()

"""영상 채널용 JPEG 생성 — 최신 프레임만 보관, 정해진 간격(기본 0.2초)으로 320×240 품질 50 JPEG (F11)."""
import numpy as np


def to_bgr(encoding: str, height: int, width: int, step: int, data) -> np.ndarray:
    """sensor_msgs/Image → OpenCV BGR (행 끝 여백 step 처리). 03_setup/camera/camera_topic_check.py와 같은 규칙."""
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
        return cv2.cvtColor(raw[:, : width * 2].reshape(height, width, 2), cv2.COLOR_YUV2BGR_YUYV)
    if enc in ("uyvy", "yuv422"):
        return cv2.cvtColor(raw[:, : width * 2].reshape(height, width, 2), cv2.COLOR_YUV2BGR_UYVY)
    raise ValueError(f"변환하지 않는 인코딩: {encoding}")


class VideoStreamer:
    def __init__(self, fps: float = 5.0, size=(320, 240), quality: int = 50):
        self.interval = 1.0 / fps
        self.size = tuple(size)
        self.quality = quality
        self._latest = None          # (encoding, h, w, step, data) — 변환은 보낼 때만
        self._next = 0.0

    def update(self, encoding, height, width, step, data) -> None:
        self._latest = (encoding, height, width, step, data)

    def next_frame(self, now: float) -> bytes | None:
        """보낼 차례이고 새 프레임이 있으면 JPEG, 아니면 None."""
        if self._latest is None or now < self._next:
            return None
        import cv2
        self._next = now + self.interval
        frame, self._latest = self._latest, None
        img = cv2.resize(to_bgr(*frame), self.size, interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        return buf.tobytes() if ok else None

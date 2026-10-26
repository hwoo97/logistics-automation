"""블루투스 채널 2 영상 프레임 — bluetooth_protocol.md 4절.

프레임 = 길이 4바이트(big-endian) + JPEG.
"""
import struct

MAX_FRAME_BYTES = 2_000_000       # 320×240 JPEG는 수십 KB — 이보다 크면 스트림이 어긋난 것


def pack_frame(jpeg: bytes) -> bytes:
    return struct.pack(">I", len(jpeg)) + jpeg


class FrameReader:
    """받은 조각을 모아 완성된 JPEG 목록을 돌려준다."""

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[bytes]:
        self._buf += data
        frames = []
        while len(self._buf) >= 4:
            (size,) = struct.unpack(">I", self._buf[:4])
            if size > MAX_FRAME_BYTES:
                self._buf.clear()
                raise ValueError(f"프레임 길이 이상: {size}")
            if len(self._buf) < 4 + size:
                break
            frames.append(bytes(self._buf[4:4 + size]))
            del self._buf[:4 + size]
        return frames

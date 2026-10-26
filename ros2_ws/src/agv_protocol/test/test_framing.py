import struct

import pytest

from agv_protocol.framing import MAX_FRAME_BYTES, FrameReader, pack_frame


def test_roundtrip_in_pieces():
    frames = [b"\xff\xd8abc\xff\xd9", b"", b"x" * 5000]
    stream = b"".join(pack_frame(f) for f in frames)
    reader, got = FrameReader(), []
    for i in range(0, len(stream), 7):          # 7바이트씩 잘라 넣기
        got += reader.feed(stream[i:i + 7])
    assert got == frames


def test_partial_frame_waits():
    reader = FrameReader()
    data = pack_frame(b"hello")
    assert reader.feed(data[:6]) == []
    assert reader.feed(data[6:]) == [b"hello"]


def test_oversized_length_rejected():
    with pytest.raises(ValueError):
        FrameReader().feed(struct.pack(">I", MAX_FRAME_BYTES + 1))

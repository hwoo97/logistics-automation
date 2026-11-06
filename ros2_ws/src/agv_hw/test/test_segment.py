"""7세그먼트 — 글자 → 세그먼트 표, LgpioSegmentDriver 핀 출력 (가짜 lgpio, Pi 없이)."""
import pytest

from agv_hw.drivers import segment
from agv_hw.drivers.fake import FakeSegmentDriver
from agv_hw.logic import SEGMENTS, segment_bits


class FakeLgpio:
    class error(Exception):
        pass

    def __init__(self):
        self.level = {}
        self.claimed = set()

    def gpio_claim_output(self, h, pin, level):
        self.claimed.add(pin)
        self.level[pin] = level

    def gpio_write(self, h, pin, level):
        self.level[pin] = level

    def gpio_free(self, h, pin):
        self.claimed.discard(pin)

    def gpiochip_close(self, h):
        pass


@pytest.mark.parametrize("char, lit", [("8", 7), ("1", 2), ("7", 3), ("0", 6), ("-", 1), (" ", 0), ("E", 5)])
def test_segment_counts(char, lit):
    assert sum(segment_bits(char)) == lit


def test_all_digits_distinct_and_unknown_is_none():
    digits = [segment_bits(str(d)) for d in range(10)]
    assert len(set(digits)) == 10 and all(len(b) == 7 for b in SEGMENTS.values())
    assert segment_bits("X") is None
    assert segment_bits("") == segment_bits(" ")


@pytest.fixture
def fake(monkeypatch):
    lg = FakeLgpio()
    monkeypatch.setattr(segment, "open_header_chip", lambda chip=None: (lg, 0))
    return lg


def test_common_cathode_high_is_on(fake):
    d = segment.LgpioSegmentDriver()
    assert fake.claimed == set(segment.DEFAULT_PINS)
    d.set(segment_bits("1"))                                # b·c만 켬
    assert [fake.level[p] for p in segment.DEFAULT_PINS] == [0, 1, 1, 0, 0, 0, 0]


def test_common_anode_is_inverted(fake):
    d = segment.LgpioSegmentDriver(common_anode=True)
    assert all(fake.level[p] == 1 for p in segment.DEFAULT_PINS)   # 시작은 모두 꺼짐 (HIGH)
    d.set(segment_bits("1"))
    assert [fake.level[p] for p in segment.DEFAULT_PINS] == [1, 0, 0, 1, 1, 1, 1]


def test_close_turns_off_and_frees(fake):
    d = segment.LgpioSegmentDriver()
    d.set(segment_bits("8"))
    d.close()
    assert not fake.claimed and all(v == 0 for v in fake.level.values())


def test_fake_driver_records():
    d = FakeSegmentDriver()
    d.set(segment_bits("3"))
    d.close()
    assert d.history[0] == ("set", segment_bits("3")) and d.bits == (0,) * 7

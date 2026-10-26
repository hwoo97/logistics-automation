"""I2C · PCA9685 점검 — Raspberry Pi 5(Ubuntu)에서 실행.

1) I2C 버스를 훑어 응답하는 주소를 보여주고  2) PCA9685(기본 0x40)의 레지스터를 읽고
3) --channel 을 주면 그 채널에 연결된 서보를 50Hz로 각도 이동시킨다(팬틸트 헤더 번호 확인용).

    python3 pca9685_check.py                          # 스캔 + 레지스터 확인만 (서보 안 움직임)
    python3 pca9685_check.py --channel 0              # 채널 0 서보: 90° → 60° → 120° → 90°
    python3 pca9685_check.py --channel 0 --oe-low     # 서보가 안 움직이면: GPIO17(OE 추정)을 LOW로
"""
import argparse
import time

MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
OSC_HZ = 25_000_000
SERVO_HZ = 50
PULSE_MIN_US, PULSE_MAX_US = 500, 2500        # SG90 0°~180° (S0에서 실제 범위 보정)
OE_PIN = 17                                   # pinmap.md: 예약, PCA9685 OE 가능성


def prescale_for(freq_hz):
    """PCA9685 PRE_SCALE 값 = round(25MHz / (4096 × 주파수)) − 1."""
    return round(OSC_HZ / (4096 * freq_hz)) - 1


def angle_to_counts(angle, freq_hz=SERVO_HZ):
    """서보 각도(0~180°) → 12비트 OFF 카운트."""
    angle = max(0.0, min(180.0, angle))
    pulse_us = PULSE_MIN_US + (PULSE_MAX_US - PULSE_MIN_US) * angle / 180.0
    period_us = 1_000_000 / freq_hz
    return round(pulse_us / period_us * 4096)


def open_bus(bus_no):
    try:
        from smbus2 import SMBus
    except ImportError:
        try:
            from smbus import SMBus
        except ImportError:
            raise SystemExit("smbus2 없음 → sudo apt install python3-smbus2 (또는 python3-smbus)")
    try:
        return SMBus(bus_no)
    except (FileNotFoundError, PermissionError) as e:
        raise SystemExit(f"/dev/i2c-{bus_no} 열기 실패: {e} → README 2절(I2C 활성화·i2c 그룹)")


def scan(bus):
    """0x03~0x77 중 응답하는 주소 목록."""
    found = []
    for addr in range(0x03, 0x78):
        try:
            bus.read_byte(addr)
            found.append(addr)
        except OSError:
            pass
    return found


class Pca9685:
    def __init__(self, bus, addr):
        self.bus, self.addr = bus, addr

    def read(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def write(self, reg, value):
        self.bus.write_byte_data(self.addr, reg, value)

    def set_freq(self, freq_hz):
        old = self.read(MODE1) & 0x7F                 # RESTART 비트는 쓰지 않음
        self.write(MODE1, old | 0x10)                 # SLEEP 상태에서만 PRESCALE 변경 가능
        self.write(PRESCALE, prescale_for(freq_hz))
        awake = (old & ~0x10) | 0x20                  # 깨우기 + 자동 증가(AI)
        self.write(MODE1, awake)
        time.sleep(0.001)                             # 발진기 안정화 500us 이상
        self.write(MODE1, awake | 0x80)               # RESTART

    def set_counts(self, channel, on, off):
        base = LED0_ON_L + 4 * channel
        for i, v in enumerate((on & 0xFF, on >> 8, off & 0xFF, off >> 8)):
            self.write(base + i, v)

    def full_off(self, channel):
        self.set_counts(channel, 0, 0x1000)               # OFF_H bit4 = 항상 꺼짐 (서보 힘 풀림)


def drive_oe_low():
    """GPIO17을 LOW로 유지 (PCA9685 OE는 LOW일 때 출력 활성). 반환값: 해제 함수."""
    import lgpio
    from led_check import find_header_chip
    chip, _ = find_header_chip(lgpio)
    h = lgpio.gpiochip_open(chip)
    lgpio.gpio_claim_output(h, OE_PIN, 0)
    print(f"GPIO{OE_PIN} LOW (gpiochip{chip})")

    def release():
        lgpio.gpio_free(h, OE_PIN)
        lgpio.gpiochip_close(h)
    return release


def main():
    parser = argparse.ArgumentParser(description="I2C 스캔 · PCA9685 · 서보 점검")
    parser.add_argument("--bus", type=int, default=1, help="I2C 버스 번호 (Pi 헤더 GPIO2·3 = 1)")
    parser.add_argument("--addr", type=lambda s: int(s, 0), default=0x40, help="PCA9685 주소 (기본 0x40)")
    parser.add_argument("--channel", type=int, choices=range(16), help="서보를 움직일 채널 0~15 (생략 시 안 움직임)")
    parser.add_argument("--angles", type=float, nargs="+", default=[90, 60, 120, 90])
    parser.add_argument("--oe-low", action="store_true", help="GPIO17을 LOW로 두고 시험")
    args = parser.parse_args()

    bus = open_bus(args.bus)
    release_oe = None
    try:
        found = scan(bus)
        print(f"I2C 버스 {args.bus} 응답 주소: {' '.join(f'0x{a:02X}' for a in found) or '없음'}")
        if args.addr not in found:
            raise SystemExit(f"0x{args.addr:02X} 응답 없음 → 제어보드 전원(배터리 스위치), 배선, 주소 확인")

        pca = Pca9685(bus, args.addr)
        mode1, mode2, pre = pca.read(MODE1), pca.read(MODE2), pca.read(PRESCALE)
        print(f"PCA9685 0x{args.addr:02X}: MODE1=0x{mode1:02X} MODE2=0x{mode2:02X} PRESCALE={pre}")

        if args.channel is None:
            print("\n[판정] PCA9685 응답 확인 → 서보 시험은 --channel N")
            return

        if args.oe_low:
            release_oe = drive_oe_low()
        pca.set_freq(SERVO_HZ)
        print(f"PRESCALE={pca.read(PRESCALE)} (50Hz 기대값 {prescale_for(SERVO_HZ)})")
        for angle in args.angles:
            counts = angle_to_counts(angle)
            print(f"채널 {args.channel}: {angle:.0f}° (OFF 카운트 {counts})")
            pca.set_counts(args.channel, 0, counts)
            time.sleep(1.0)
        pca.full_off(args.channel)
        print(f"\n[판정] 채널 {args.channel}에 연결된 서보(헤더 번호·팬/틸트)를 README 결과표에 기록")
    finally:
        if release_oe:
            release_oe()
        bus.close()


if __name__ == "__main__":
    main()

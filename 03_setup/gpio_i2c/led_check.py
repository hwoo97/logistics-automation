"""lgpio LED 점등 점검 — Raspberry Pi 5(Ubuntu)에서 실행.

헤더 GPIO 칩(RP1)을 자동으로 찾아 빨강(GPIO16)·초록(GPIO26) LED를 차례로 깜박인다.
핀은 02_design/hardware/wiring.md 기준. 결과는 눈으로 확인한다.

    python3 led_check.py
    python3 led_check.py --pins 16 --blinks 5
"""
import argparse
import time

LED_PINS = {16: "빨강(원)", 26: "초록(사각형)"}


def find_header_chip(lgpio, max_chip=8):
    """라벨에 'rp1'이 들어간 gpiochip 번호를 찾는다 (Pi 5는 커널에 따라 0 또는 4)."""
    found = []
    for n in range(max_chip):
        try:
            h = lgpio.gpiochip_open(n)
        except lgpio.error:
            continue
        try:
            info = lgpio.gpio_get_chip_info(h)      # (상태, 라인 수, 이름, 라벨)
            found.append((n, info[1], info[2], info[3]))
        finally:
            lgpio.gpiochip_close(h)
    for n, lines, name, label in found:
        if "rp1" in str(label).lower():
            return n, found
    return None, found


def main():
    parser = argparse.ArgumentParser(description="LED 점등 점검 (lgpio)")
    parser.add_argument("--chip", type=int, help="gpiochip 번호 직접 지정 (생략 시 자동 탐색)")
    parser.add_argument("--pins", type=int, nargs="+", default=list(LED_PINS), help="BCM 핀 (기본 16 26)")
    parser.add_argument("--blinks", type=int, default=3)
    parser.add_argument("--period", type=float, default=0.5, help="켜짐/꺼짐 시간 초")
    args = parser.parse_args()

    try:
        import lgpio
    except ImportError:
        raise SystemExit("lgpio 없음 → sudo apt install python3-lgpio")

    chip = args.chip
    if chip is None:
        chip, found = find_header_chip(lgpio)
        for n, lines, name, label in found:
            print(f"  gpiochip{n}: {name} / {label} / 라인 {lines}")
        if chip is None:
            raise SystemExit("RP1 헤더 GPIO 칩을 찾지 못함 → --chip 으로 지정 (gpiodetect 참고)")
    print(f"헤더 GPIO 칩: gpiochip{chip}")

    try:
        h = lgpio.gpiochip_open(chip)
    except lgpio.error as e:
        raise SystemExit(f"gpiochip{chip} 열기 실패: {e} → /dev/gpiochip{chip} 권한(그룹) 확인")
    try:
        for pin in args.pins:
            lgpio.gpio_claim_output(h, pin, 0)
        for pin in args.pins:
            name = LED_PINS.get(pin, "")
            print(f"GPIO{pin} {name} 깜박임 {args.blinks}회")
            for _ in range(args.blinks):
                lgpio.gpio_write(h, pin, 1)
                time.sleep(args.period)
                lgpio.gpio_write(h, pin, 0)
                time.sleep(args.period)
        print("모두 켜기 2초")
        for pin in args.pins:
            lgpio.gpio_write(h, pin, 1)
        time.sleep(2)
    finally:
        for pin in args.pins:
            try:
                lgpio.gpio_write(h, pin, 0)
                lgpio.gpio_free(h, pin)
            except lgpio.error:
                pass
        lgpio.gpiochip_close(h)
    print("\n[판정] 각 LED가 이름대로 깜박였는지 눈으로 확인 → README 결과표에 기록")


if __name__ == "__main__":
    main()

"""HC-SR04 초음파 점검 — Raspberry Pi 5(Ubuntu)에서 실행. 노드(launch)를 끄고 실행한다 (같은 핀 사용).

같은 센서를 두 방식으로 번갈아 재서 원인을 나눈다.
  콜백 — ultrasonic_node가 쓰는 방식 (lgpio 경계 콜백 타임스탬프)
  읽기 — Echo 핀을 반복해서 읽어 펄스 폭을 잰다 (콜백과 무관)
Echo 대기 상태(트리거 전 레벨)도 함께 보여 준다 — 계속 1이면 배선·분압 문제.

    python3 ultrasonic_check.py              # 0.2초마다 20회
    python3 ultrasonic_check.py --count 50 --trig 5 --echo 6
"""
import argparse
import threading
import time

SPEED_OF_SOUND = 343.0


def find_header_chip(lgpio, max_chip=8):
    for n in range(max_chip):
        try:
            h = lgpio.gpiochip_open(n)
        except lgpio.error:
            continue
        label = str(lgpio.gpio_get_chip_info(h)[3]).lower()
        lgpio.gpiochip_close(h)
        if "rp1" in label:
            return n
    return None


def trigger(lgpio, h, trig):
    lgpio.gpio_write(h, trig, 1)
    time.sleep(0.00001)
    lgpio.gpio_write(h, trig, 0)


def measure_poll(lgpio, h, trig, echo, timeout=0.03):
    """Echo 핀 반복 읽기 — 상승·하강 시각을 perf_counter_ns로."""
    trigger(lgpio, h, trig)
    t0 = time.perf_counter()
    while lgpio.gpio_read(h, echo) == 0:
        if time.perf_counter() - t0 > timeout:
            return None, "상승 없음"
    rise = time.perf_counter_ns()
    while lgpio.gpio_read(h, echo) == 1:
        if time.perf_counter() - t0 > timeout:
            return None, "하강 없음(너무 멀거나 Echo 고정)"
    width = time.perf_counter_ns() - rise
    return width * 1e-9 * SPEED_OF_SOUND / 2, ""


class CallbackMeter:
    def __init__(self, lgpio, h, echo):
        self.rise = self.width = None
        self.done = threading.Event()
        self.edges = 0
        self.cb = lgpio.callback(h, echo, lgpio.BOTH_EDGES, self.on_edge)

    def on_edge(self, chip, gpio, level, tick):
        self.edges += 1
        if level == 1:
            self.rise = tick
        elif level == 0 and self.rise is not None:
            self.width = tick - self.rise
            self.done.set()

    def measure(self, lgpio, h, trig, timeout=0.03):
        self.rise = self.width = None
        self.done.clear()
        trigger(lgpio, h, trig)
        if not self.done.wait(timeout):
            return None, "시간 초과(콜백 안 옴)"
        return self.width * 1e-9 * SPEED_OF_SOUND / 2, ""


def fmt(value, why):
    return f"{value * 100:6.1f} cm" if value is not None else f"  실패 ({why})"


def main():
    ap = argparse.ArgumentParser(description="HC-SR04 초음파 점검 (lgpio)")
    ap.add_argument("--chip", type=int)
    ap.add_argument("--trig", type=int, default=5)
    ap.add_argument("--echo", type=int, default=6)
    ap.add_argument("--count", type=int, default=20)
    ap.add_argument("--period", type=float, default=0.2)
    args = ap.parse_args()

    import lgpio
    chip = args.chip if args.chip is not None else find_header_chip(lgpio)
    h = lgpio.gpiochip_open(chip)
    lgpio.gpio_claim_output(h, args.trig, 0)
    lgpio.gpio_claim_alert(h, args.echo, lgpio.BOTH_EDGES)
    meter = CallbackMeter(lgpio, h, args.echo)
    print(f"gpiochip{chip}, Trig GPIO{args.trig}, Echo GPIO{args.echo} — 손을 10cm·30cm·치움으로 바꿔 가며 보기")
    print(" #  Echo대기  콜백              읽기")
    ok_cb = ok_poll = 0
    try:
        for i in range(args.count):
            idle = lgpio.gpio_read(h, args.echo)
            cb = meter.measure(lgpio, h, args.trig)
            time.sleep(0.06)                                  # 앞 펄스 끝날 시간
            poll = measure_poll(lgpio, h, args.trig, args.echo)
            ok_cb += cb[0] is not None
            ok_poll += poll[0] is not None
            print(f"{i + 1:2d}  {idle:^7d}  {fmt(*cb):16s}  {fmt(*poll)}")
            time.sleep(args.period)
    except KeyboardInterrupt:
        pass
    finally:
        meter.cb.cancel()
        lgpio.gpiochip_close(h)
    print(f"성공: 콜백 {ok_cb}/{args.count}, 읽기 {ok_poll}/{args.count}, 콜백 경계 수신 {meter.edges}개")
    if ok_poll and not ok_cb:
        print("→ 센서·배선은 정상, 콜백 방식 문제 — 노드를 읽기 방식으로 바꿔야 함")
    elif not ok_poll and not ok_cb:
        print("→ 둘 다 실패: Trig·Echo 배선, 5V 전원, Echo 분압(1k+2k) 확인")


if __name__ == "__main__":
    main()

"""RFCOMM 에코 클라이언트 — 노트북(Windows/Ubuntu)에서 실행.

1) 연결 시간  2) 에코 왕복 시간(RTT)  3) Pi → 노트북 처리량을 측정하고
블루투스 규약의 기준(하트비트 1.5초, 영상 320×240 JPEG 5fps)과 비교한다.
표준 라이브러리만 사용. 사용법은 같은 폴더 README.md 참고.

    python rfcomm_echo_client.py AA:BB:CC:DD:EE:FF
    python rfcomm_echo_client.py AA:BB:CC:DD:EE:FF --channel 2 --count 200 --bulk 2000000
    python rfcomm_echo_client.py --tcp 127.0.0.1:9000      # 블루투스 없이 로직 확인용
"""
import argparse
import socket
import statistics
import time

HEARTBEAT_TIMEOUT_S = 1.5          # bluetooth_protocol.md 5절
VIDEO_NEED_KBPS = 15 * 8 * 5       # 320×240 품질 50 JPEG 약 15KB × 5fps ≈ 600kbps (추정)


def connect(args):
    if args.tcp:
        host, port = args.tcp.rsplit(":", 1)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        target = (host, int(port))
    else:
        sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        target = (args.address, args.channel)
    sock.settimeout(args.timeout)
    start = time.perf_counter()
    sock.connect(target)
    return sock, time.perf_counter() - start


def recv_line(sock, buf):
    while b"\n" not in buf:
        data = sock.recv(1024)
        if not data:
            raise ConnectionError("서버가 연결을 끊음")
        buf += data
    line, buf = buf.split(b"\n", 1)
    return line, buf


def echo_test(sock, count, interval):
    rtts, lost = [], 0
    buf = b""
    for i in range(count):
        msg = f"PING {i}".encode()
        start = time.perf_counter()
        sock.sendall(msg + b"\n")
        try:
            line, buf = recv_line(sock, buf)
        except socket.timeout:
            # 응답이 제한 시간 안에 없으면 연결이 끊긴 것으로 보고 중단 (계속 기다리면 채널을 붙잡고 멈춤)
            lost += count - i
            print(f"PING {i} 응답 없음 → 연결 끊김으로 보고 중단")
            break
        rtt = (time.perf_counter() - start) * 1000
        if line == msg:
            rtts.append(rtt)
        else:
            lost += 1
        time.sleep(interval)
    return rtts, lost


def bulk_test(sock, n):
    sock.sendall(f"BULK {n}\n".encode())
    got = 0
    start = time.perf_counter()
    while got < n:
        data = sock.recv(min(65536, n - got))
        if not data:
            raise ConnectionError("BULK 수신 중 연결 끊김")
        got += len(data)
    elapsed = time.perf_counter() - start
    return elapsed, got * 8 / 1000 / elapsed


def main():
    parser = argparse.ArgumentParser(description="RFCOMM 에코·처리량 측정")
    parser.add_argument("address", nargs="?", help="Pi 블루투스 주소 (bluetoothctl show 의 Controller 주소)")
    parser.add_argument("--channel", type=int, default=1, help="RFCOMM 채널 (기본 1)")
    parser.add_argument("--count", type=int, default=100, help="에코 횟수 (기본 100)")
    parser.add_argument("--interval", type=float, default=0.05, help="에코 간격 초 (기본 0.05)")
    parser.add_argument("--bulk", type=int, default=1_000_000, help="처리량 측정 바이트 수, 0이면 생략 (기본 1MB)")
    parser.add_argument("--timeout", type=float, default=10.0, help="연결·수신 제한 시간 초")
    parser.add_argument("--tcp", help="블루투스 대신 host:port 로 연결 (로직 확인용)")
    args = parser.parse_args()
    if not args.tcp and not args.address:
        parser.error("Pi 블루투스 주소 또는 --tcp 를 지정")

    try:
        sock, t_conn = connect(args)
    except OSError as e:
        print(f"연결 실패: {e}")
        if getattr(e, "winerror", None) == 10048:
            print("→ 이 PC에서 이전 클라이언트가 같은 채널 연결을 아직 잡고 있음. 그 프로그램을 종료한 뒤 다시 실행")
        else:
            print("→ Pi 서버가 실행 중인지, 채널 번호·주소·페어링 상태 확인")
        raise SystemExit(1)
    print(f"연결 성공 ({t_conn:.2f}초)")
    try:
        run_tests(sock, args)
    except (OSError, ConnectionError) as e:
        print(f"시험 중 연결 끊김: {e}")
        raise SystemExit(1)
    finally:
        sock.close()


def run_tests(sock, args):
    rtts, lost = echo_test(sock, args.count, args.interval)
    print(f"에코 {len(rtts)}/{args.count} 성공, 유실 {lost}")
    if rtts:
        rtts_sorted = sorted(rtts)
        p95 = rtts_sorted[max(0, int(len(rtts_sorted) * 0.95) - 1)]
        print(f"RTT(ms) 최소 {min(rtts):.1f} / 평균 {statistics.mean(rtts):.1f} / "
              f"95% {p95:.1f} / 최대 {max(rtts):.1f}")
    kbps = None
    if args.bulk and lost == 0:
        elapsed, kbps = bulk_test(sock, args.bulk)
        print(f"처리량 Pi→노트북: {args.bulk}바이트 {elapsed:.2f}초 = {kbps:.0f} kbps")
    sock.sendall(b"QUIT\n")

    print("\n[판정]")
    ok_echo = bool(rtts) and lost == 0
    ok_rtt = bool(rtts) and max(rtts) < HEARTBEAT_TIMEOUT_S * 1000 / 3
    print(f"- 에코 유실 0건: {'통과' if ok_echo else '실패'}")
    print(f"- 최대 RTT < {HEARTBEAT_TIMEOUT_S * 1000 / 3:.0f}ms (하트비트 1.5초의 1/3): {'통과' if ok_rtt else '실패'}")
    if kbps is not None:
        verdict = "충분" if kbps >= VIDEO_NEED_KBPS * 1.5 else ("빠듯" if kbps >= VIDEO_NEED_KBPS else "부족")
        print(f"- 영상 필요 약 {VIDEO_NEED_KBPS}kbps 대비: {verdict} (여유 1.5배 이상이면 충분)")


if __name__ == "__main__":
    main()

"""RFCOMM 에코 서버 — Raspberry Pi 5(Ubuntu)에서 실행.

받은 줄을 그대로 돌려보내고, `BULK <n>` 요청에는 n바이트를 연속 전송한다(Pi → 노트북 처리량 측정).
표준 라이브러리만 사용. 사용법은 같은 폴더 README.md 참고.

    python3 rfcomm_echo_server.py              # RFCOMM 채널 1
    python3 rfcomm_echo_server.py --channel 2
    python3 rfcomm_echo_server.py --tcp 9000   # 블루투스 없이 로직 확인용
"""
import argparse
import socket
import time

BULK_CHUNK = 4096


def make_server(args):
    if args.tcp:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", args.tcp))
        where = f"TCP 포트 {args.tcp}"
    else:
        srv = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        srv.bind((socket.BDADDR_ANY, args.channel))
        where = f"RFCOMM 채널 {args.channel}"
    srv.listen(1)
    return srv, where


def send_bulk(conn, n):
    chunk = b"\x55" * BULK_CHUNK
    sent = 0
    start = time.perf_counter()
    while sent < n:
        part = chunk[: min(BULK_CHUNK, n - sent)]
        conn.sendall(part)
        sent += len(part)
    return time.perf_counter() - start


def serve(conn):
    buf = b""
    lines = 0
    while True:
        data = conn.recv(1024)
        if not data:
            return lines
        buf += data
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode("utf-8", errors="replace").strip()
            if text.startswith("BULK "):
                n = int(text.split()[1])
                elapsed = send_bulk(conn, n)
                print(f"  BULK {n}바이트 전송 ({elapsed:.2f}초)")
            elif text == "QUIT":
                return lines
            else:
                conn.sendall(line + b"\n")
                lines += 1


def main():
    parser = argparse.ArgumentParser(description="RFCOMM 에코 서버")
    parser.add_argument("--channel", type=int, default=1, help="RFCOMM 채널 (기본 1)")
    parser.add_argument("--tcp", type=int, help="블루투스 대신 TCP 포트로 대기 (로직 확인용)")
    args = parser.parse_args()

    srv, where = make_server(args)
    print(f"{where}에서 연결 대기 중... (Ctrl+C 종료)")
    try:
        while True:
            conn, addr = srv.accept()
            print(f"연결됨: {addr}")
            with conn:
                try:
                    lines = serve(conn)
                except OSError as e:
                    lines = None
                    print(f"  연결 오류: {e}")
            print(f"연결 종료: {addr} (에코 {lines}줄)")
    except KeyboardInterrupt:
        pass
    finally:
        srv.close()


if __name__ == "__main__":
    main()

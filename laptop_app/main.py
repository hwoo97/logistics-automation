"""노트북 앱 실행 — 블루투스로 차량에 연결해 사용자 탭(운행)·개발자 탭(시험)을 제공한다.

    python main.py                       # 블루투스 (Pi 주소는 화면에서 입력, config.json에 저장)
    python main.py --addr 2C:CF:67:2C:DB:A9
    python main.py --fake                # Pi 없이: 가짜 차량을 함께 띄워 PC에서 시험
"""
import argparse
import os
import subprocess
import sys
import time
import tkinter as tk

import _paths  # noqa: F401
from ui.main_window import MainWindow

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    parser = argparse.ArgumentParser(description="물류 AGV 노트북 앱")
    parser.add_argument("--addr", help="Pi 블루투스 주소")
    parser.add_argument("--fake", action="store_true", help="가짜 차량을 띄우고 그쪽으로 연결")
    parser.add_argument("--port", type=int, default=9000, help="가짜 차량 포트 (영상 +1)")
    args = parser.parse_args()

    fake_proc = None
    if args.fake:
        fake_proc = subprocess.Popen([sys.executable, os.path.join(HERE, "fake_vehicle.py"), "--port", str(args.port)],
                                     cwd=HERE)
        time.sleep(1.0)

    root = tk.Tk()
    try:
        MainWindow(root, fake=args.fake, addr=args.addr, fake_port=args.port)
        root.mainloop()
    finally:
        if fake_proc:
            fake_proc.terminate()


if __name__ == "__main__":
    main()

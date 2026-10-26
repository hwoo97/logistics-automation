"""레포 안의 공용 패키지(agv_protocol, agv_bt)를 설치 없이 import할 수 있게 경로를 추가한다.

`pip install -e ../ros2_ws/src/agv_protocol`로 설치했다면 없어도 되지만, 레포를 받기만 해도 실행되도록 둔다.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for pkg in ("agv_protocol", "agv_bt"):
    path = os.path.join(ROOT, "ros2_ws", "src", pkg)
    if path not in sys.path:
        sys.path.insert(0, path)

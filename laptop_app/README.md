# 노트북 앱 (Python · tkinter)

블루투스(RFCOMM)로 차량(Pi 5)에 연결해 **사용자 탭**(실제 운행)과 **개발자 탭**(시험·점검)을 제공한다. PC에는 ROS가 필요 없다.
통신 규약: [bluetooth_protocol.md](../02_design/communication/bluetooth_protocol.md) · 메시지 코드는 차량과 공용 [`agv_protocol`](../ros2_ws/src/agv_protocol)

## 실행
```bash
cd laptop_app
pip install -r requirements.txt          # Pillow (영상 표시)
python main.py --fake                    # Pi 없이: 가짜 차량을 함께 띄워 PC에서 시험
python main.py                           # 실제 차량: 화면에서 "블루투스 (Pi)" + Pi 주소 입력 → 연결
```
- Python 3.10 이상 (Windows는 python.org 설치본 — 블루투스 소켓·tkinter 포함)
- Pi 주소와 영상 반전 설정은 `config.json`에 저장된다 (GitHub에 올리지 않음)
- `agv_protocol`은 레포 경로에서 자동으로 불러온다 (`_paths.py`). 설치해서 쓰려면 `pip install -e ../ros2_ws/src/agv_protocol`

## 화면
| 위치 | 내용 |
|------|------|
| 상단 (공통) | 연결 방식(블루투스 / 가짜 차량), Pi 주소, 연결·해제, RTT, **영상 상하·좌우 반전**(두 탭 화면에만 적용, 연결 중에도 변경 가능), **비상 정지(Space)** |
| 사용자 탭 | 작업 횟수 → 작업 시작, 재개, 상태(대기·이동·적재…)·진행·화물·목적지, 수동 제어(W/A/S/D), 사건·오류, 영상 |
| 개발자 탭 | LED, 팬틸트 자세·각도, 바퀴 4개 개별 듀티 / 초음파 거리 그래프, 라인 오프셋, Pi 온도·CPU, ROS 노드 목록 / 영상 FPS·kbps, 사진 저장(`captures/`, 반전 전 원본 — 차량 AI가 보는 그대로), **라인 주행 시험**(▶ 출발 = 카메라 LINE 자세 후 drive_node 출발, ■ 정지, 상태 "주행 중 / 끝: STOP_LINE / 거부: …", 속도 0.2~1.0 · 회전 세기 0.1~1.5 · 라인 기준 −0.5~0.5 · 확인 프레임 1~10 · 라인 끝 프레임 3~30 — 주행 중에도 적용, config.json에 저장하고 연결할 때마다 차량에 다시 보냄) / 메시지 콘솔(직접 입력, 로그 저장) |

- 개발자 명령(`DEV ...`)은 차량이 대기·수동·비상 정지·오류 상태일 때만 받는다 (작업 중에는 거부 `NAK`)

## 파일
| 파일 | 역할 |
|------|------|
| `main.py` | 실행 (`--fake`, `--addr`) |
| `bt_client.py` | 채널 1 연결, 수신 스레드, 0.5초 PING |
| `video_receiver.py` | 채널 2 영상 수신, FPS·kbps |
| `ui/main_window.py` | 연결 막대 + 탭, 수신 메시지 분배 |
| `ui/user_tab.py`, `ui/manual_pad.py` | 사용자 탭 |
| `ui/dev_tab.py` | 개발자 탭 |
| `ui/video_view.py` | 영상 표시 |
| `fake_vehicle.py` | 가짜 차량 — 실제와 같은 `RfcommServer`(agv_bt)·메시지 규칙으로 작업 사이클·DEV·영상 흉내 |

## 차량 쪽 상태
- 메시지 규칙 `agv_protocol`, RFCOMM 서버 `agv_bt/rfcomm_server.py` 완료 (pytest)
- 실제 차량의 `bt_bridge_node`(ROS 2)와 하드웨어 노드는 다음 단계에서 작성 → 그 전까지는 가짜 차량으로 화면 시험

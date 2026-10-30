# ROS 2 워크스페이스 (Pi 5 · Ubuntu 24.04 · ROS 2 Jazzy · Python)
- 차량 코드는 `src/` 아래 ROS 2 패키지로 작성한다: agv_interfaces · agv_protocol · agv_hw · agv_vision · agv_control · agv_bt · agv_bringup
- 패키지·모듈·클래스 구조: [software_architecture.md](../02_design/software/software_architecture.md) · 노드·토픽·상태 머신: [functional_design.md](../02_design/software/functional_design.md)

## 현재 작성된 패키지
| 패키지 | 내용 | 시험 |
|------|------|------|
| `agv_protocol` | BT 메시지·영상 프레임·공통 타입 (순수 Python, 노트북 앱과 공용) | pytest 56 |
| `agv_hw` | 드라이버(모터 DRV8835·서보 PCA9685·초음파·LED, 가짜 포함) + 노드 `motor_node` `pantilt_node` `ultrasonic_node` `led_node` | pytest 21 (모터 드라이버 4 포함) |
| `agv_bt` | `RfcommServer`, `BridgeCore`(명령 판단), `VideoStreamer`, `SysMonitor` + 노드 `bt_bridge_node` | pytest 14 (RfcommServer 4 포함) |
| `agv_interfaces` | 메시지 `LineState` (offset, line_found, event, raw_event) — Status·Command·Drive는 상태 머신 작업 때 | — |
| `agv_vision` | `LineDetector`(검정 = 어둡고 색 없음, 가운데·직전 위치·아래 닿음 가중치로 라인 고르기, 이어진 영역만으로 이벤트, 3프레임 확인) + 노드 `line_node` | pytest 21 (합성 트랙 사진) |
| `agv_bringup` | `launch/s0_hw.launch.py`, `launch/s1_line.launch.py`(S0 + line_node), `config/params.yaml` | — |

## 빌드 (Pi)
```bash
sudo apt install -y python3-lgpio python3-smbus2 python3-opencv python3-numpy python3-pytest
cd ~/logistics-automation/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/camera_ws/install/setup.bash          # camera_ros (03_setup/camera)
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select agv_protocol agv_hw agv_bt agv_vision && colcon test-result --verbose
```

## S0 기초 동작 — 노트북 앱 개발자 탭으로 시험
1. **가짜 드라이버로 먼저** (하드웨어 안 움직임, 노드·토픽 연결 확인)
   ```bash
   ros2 launch agv_bringup s0_hw.launch.py use_fake_hw:=true camera:=false
   ros2 node list          # 5개: motor_node pantilt_node ultrasonic_node led_node bt_bridge_node
   ```
2. **실제 하드웨어** — 차량을 받침대에 올려 바퀴를 띄운 상태에서
   ```bash
   ros2 launch agv_bringup s0_hw.launch.py
   ```
3. 노트북: `python laptop_app/main.py` → "블루투스 (Pi)" + Pi 주소 → 연결 → **개발자 탭**
   | 확인 | 방법 | 결과 반영 |
   |------|------|------|
   | LED | RED / GREEN / ERROR(깜빡임) / OFF | — |
   | 팬틸트 | LINE·JUNCTION·SIGN 버튼, 팬·틸트 슬라이더 | `poses` 각도 |
   | 바퀴 위치·방향 | L1~L4 슬라이더 하나씩 0.3 → "보내기" → 어느 바퀴가 어느 방향으로 도는지 | `wheel_map`, `invert` |
   | 초음파 | WATCH RANGE → 손을 가까이·멀리 | — |
   | 영상 | 영상 FPS·kbps (블루투스 실측 286kbps 참고) | `video_fps`, `jpeg_quality` |
   | 비상 정지 | 바퀴 도는 중 Space → 즉시 정지, 재개 | — |
   | 수동 | 수동 모드 + W/A/S/D (바퀴 띄운 상태) | `turn_gain` |
4. 확정 값은 `src/agv_bringup/config/params.yaml`에, 결과는 `04_unit/02_motor/test.md` 등에 기록

- `standalone: true`(기본): mission_node 작성 전이라 `START`는 거부되고, 브리지가 대기·수동·비상 정지 상태를 직접 관리한다
- 블루투스 대신 같은 네트워크에서 시험: `transport:=tcp` → 노트북 앱은 코드에서 host를 Pi IP로 (추후 화면에 추가)

## S1 라인 검출 — 노트북 앱으로 확인 (주행은 다음 단계)
```bash
cd ~/logistics-automation && git pull
cd ros2_ws && colcon build --symlink-install && source install/setup.bash   # 새 패키지(agv_interfaces·agv_vision)라 빌드 필요
ros2 launch agv_bringup s1_line.launch.py
```
- 개발자 탭: 팬틸트 **LINE** → 측정값 받기 **LINE** 체크 → "라인 오프셋·이벤트" 칸
- 확인: 라인을 화면 가운데·왼쪽·오른쪽에 둘 때 오프셋 0·음수·양수, 라인을 치우면 `LINE_END`, 옆에 색 마커를 둬도 오프셋이 흔들리지 않는지
- `(판정 …)`은 이번 프레임 판정, `이벤트`는 3프레임 연속 확인된 값
- 잘 안 맞으면 개발자 탭 "사진 저장"으로 그 장면을 저장 → PC에서 `LineDetector`로 재현해 값 조정


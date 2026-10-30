# 소프트웨어 구조 설계

> 범위: 차량(Pi 5, ROS 2 Jazzy, Python) + 노트북 앱(Python)의 **코드 구조** — 패키지·모듈·클래스, 계층, 실행 구성, 테스트·개발 방식
> 무엇을 하는지(기능·노드·토픽·상태 머신)는 [기능 설계서](functional_design.md), 메시지 형식은 [블루투스 규약](../communication/bluetooth_protocol.md), 클래스 상세는 [클래스 설계](class_design.md)

---

## 1. 설계 원칙

| 원칙 | 내용 | 이유 |
|------|------|------|
| **3계층 분리** | 하드웨어 드라이버 / 알고리즘(순수 Python) / ROS 노드(얇은 껍데기) | 알고리즘을 ROS·하드웨어 없이 노트북에서 테스트 |
| **드라이버 교체 가능** | 드라이버는 같은 인터페이스의 실제(`Lgpio…`)·가짜(`Fake…`) 두 구현 | 하드웨어 없이 노드 실행·테스트 |
| **노드 하나 = 책임 하나** | 기능 설계서의 노드 단위 그대로 | 단위 기능별로 개발·검증 (04_unit, S0~S10) |
| **값은 코드 밖으로** | 속도·임계값·각도·HSV 범위는 `params.yaml` | 테스트 트랙 보정 값을 코드 수정 없이 반영 |
| **상태 머신은 순수 로직** | 입력(이벤트) → 출력(동작 목록)만 계산, 실행은 노드가 | 시나리오를 단위 테스트로 검증 |
| **프로토콜 단일 소스** | BT 메시지 만들기·해석 코드를 한 패키지로 공유 | 차량·노트북 형식 불일치 방지 |

---

## 2. 계층 구조

```
 ┌──────────────────────────────────────────────────────────────┐
 │ ③ ROS 노드 계층   (rclpy — 구독·발행·서비스·액션·파라미터)        │
 │    motor_node  line_node  marker_node  mission_node  bt_bridge…│
 ├──────────────────────────────────────────────────────────────┤
 │ ② 알고리즘 계층   (순수 Python + OpenCV/numpy, ROS 의존 없음)     │
 │    LineDetector  MarkerDetector  CargoClassifier  PidController │
 │    MissionStateMachine  BranchSelector  agv_protocol          │
 ├──────────────────────────────────────────────────────────────┤
 │ ① 하드웨어 드라이버 계층  (인터페이스 + 실제/가짜 구현)              │
 │    MotorDriver  ServoDriver  UltrasonicDriver  LedDriver       │
 │    └ 실제: lgpio / smbus2 (PCA9685)   └ 가짜: 메모리 기록         │
 └──────────────────────────────────────────────────────────────┘
```
- 노드는 ①을 생성해 ②에 값을 넘기고, ②의 결과를 토픽·명령으로 내보내는 일만 한다
- ② 계층 코드는 노트북(Windows)에서도 `pytest`로 실행 가능

---

## 3. 저장소 구조

```
logistics-automation/
├─ ros2_ws/src/
│  ├─ agv_interfaces/               # ament_cmake — 메시지·서비스·액션 정의
│  │  ├─ msg/  LineState.msg  Status.msg  Command.msg
│  │  ├─ srv/  DetectMarkers.srv  Classify.srv
│  │  ├─ action/  Drive.action
│  │  ├─ CMakeLists.txt  package.xml
│  │
│  ├─ agv_protocol/                 # ament_python — BT 메시지 + 공통 타입 (순수 Python, 노트북도 설치)
│  │  └─ agv_protocol/  messages.py  framing.py  types.py
│  │
│  ├─ agv_hw/                       # 하드웨어 드라이버 + 노드
│  │  └─ agv_hw/
│  │     ├─ drivers/  base.py  motor.py  servo_pca9685.py  ultrasonic.py  led.py  fake.py
│  │     ├─ logic.py                # MedianFilter, mix, wheel_speeds, led_output, Watchdog (순수)
│  │     ├─ motor_node.py  pantilt_node.py  ultrasonic_node.py  led_node.py
│  │
│  ├─ agv_vision/                   # 영상 알고리즘 + 노드
│  │  └─ agv_vision/
│  │     ├─ line_detector.py  marker_detector.py  classifier.py
│  │     ├─ line_node.py  marker_node.py  classifier_node.py
│  │
│  ├─ agv_control/                  # 주행·상태 머신
│  │  └─ agv_control/
│  │     ├─ pid.py  drive_controller.py  state_machine.py  branch_selector.py
│  │     ├─ drive_node.py  mission_node.py
│  │
│  ├─ agv_bt/                       # 블루투스 브리지
│  │  └─ agv_bt/  rfcomm_server.py  bridge_core.py  video_streamer.py  sysinfo.py  bt_bridge_node.py
│  │
│  └─ agv_bringup/                  # 실행 구성
│     ├─ launch/  agv.launch.py  s0_hw.launch.py … s10_full.launch.py
│     ├─ config/  params.yaml
│     └─ models/  cargo.tflite  sign.tflite  labels_*.txt
│
├─ laptop_app/                      # 노트북 제어 앱 (ROS 없음)
│  ├─ main.py  _paths.py
│  ├─ bt_client.py  video_receiver.py
│  ├─ fake_vehicle.py              # 가짜 차량 (Pi 없이 PC 시험, RfcommServer 재사용)
│  ├─ ui/  main_window.py  user_tab.py  dev_tab.py  video_view.py  manual_pad.py
│  └─ requirements.txt
│
└─ 04_unit/<번호_기능>/             # 단위 테스트 기록(test.md) + 수동 점검 스크립트
```

- 각 Python 패키지에는 표준 파일(`package.xml`, `setup.py`, `setup.cfg`, `resource/`, `test/`)이 있다 (트리에서는 생략)
- `test/` 폴더: 패키지마다 ② 계층 단위 테스트

---

## 4. 패키지별 모듈 · 클래스

### agv_hw — 하드웨어
| 모듈 | 클래스 / 함수 | 책임 |
|------|--------------|------|
| `drivers/motor.py` | `MotorDriver`(인터페이스), `LgpioMotorDriver` | 바퀴 4개 `set(wheel, speed)`, 듀티 상한·극성 반전 적용, `stop_all()` |
| `drivers/servo_pca9685.py` | `ServoDriver`, `Pca9685ServoDriver` | 채널별 각도 → PWM, 50Hz 설정, GPIO17(OE) 처리 |
| `drivers/ultrasonic.py` | `UltrasonicDriver`, `LgpioUltrasonicDriver` | Trig 펄스 → Echo 폭 → 거리(m), 시간 초과 처리 |
| `drivers/led.py` | `LedDriver`, `LgpioLedDriver` | 빨강·초록 켜기/끄기 |
| `drivers/fake.py` | `FakeMotorDriver` 등 | 하드웨어 없이 호출 기록 (테스트·노트북 실행용) |
| `motor_node.py` | `MotorNode` | `/cmd_vel` → 좌우 속도 → 바퀴 4개, 워치독, `/safety/estop` 래치 |
| `pantilt_node.py` | `PanTiltNode` | `/pantilt/pose` → 자세별 각도, 안정화 대기 |
| `ultrasonic_node.py` | `UltrasonicNode` | 15Hz 측정, 중앙값 필터, `/ultrasonic/range` |
| `led_node.py` | `LedNode` | `/led/state` → 점등·깜빡임 타이머 |

- 노드 파라미터 `use_fake_hw: true`면 가짜 드라이버 사용

### agv_vision — 영상
| 모듈 | 클래스 / 함수 | 입력 → 출력 |
|------|--------------|-------------|
| `line_detector.py` | `LineDetector.process(bgr) -> LineResult` | 영상 → 오프셋, 라인 유무, 이벤트(STOP_LINE·JUNCTION_*·CORNER_*·LINE_END) + 연속 프레임 확정 |
| `marker_detector.py` | `MarkerDetector.detect(bgr) -> dict[str, list[str]]` | 영상 → 갈래별 색 목록, `vote(frames)` 다수결 |
| `classifier.py` | `TfliteClassifier.predict(bgr) -> (label, score)`, `decide(results)` | LiteRT 추론, 5프레임 판정 규칙 |
| `line_node.py` | `LineNode` | 영상 구독(최신 1장) → `/line/state` 발행 |
| `marker_node.py` | `MarkerNode` | `/marker/detect` 요청 시 최근 프레임 10장으로 판별 |
| `classifier_node.py` | `ClassifierNode` | `/vision/classify` 요청 시 모델(CARGO/SIGN) 선택 → 판정 |

### agv_control — 주행·상태 머신
| 모듈 | 클래스 / 함수 | 책임 |
|------|--------------|------|
| `pid.py` | `PidController` | 오프셋 → 각속도 |
| `drive_controller.py` | `DriveController` | 동작별 단계 진행(FOLLOW·TURN·UTURN·APPROACH) — 입력: 라인 상태, 출력: 속도 명령·완료 이벤트 |
| `branch_selector.py` | `choose_branch(markers, target)` | 목적지 색 갈래 선택, 정확히 1개 아니면 `MarkerError` |
| `state_machine.py` | `MissionStateMachine.handle(event) -> list[Action]` | 기능 설계서 5절 상태 머신. 이벤트(START, STOP_LINE, SIGN, RANGE, CARGO, TIMEOUT…) → 동작(drive, pantilt, led, classify, report…) |
| `drive_node.py` | `DriveNode` | `/drive` 액션 서버, `DriveController` 실행, `/cmd_vel` 발행 |
| `mission_node.py` | `MissionNode` | 토픽·서비스 결과를 이벤트로 바꿔 상태 머신에 넣고, 나온 동작을 실행 |

### agv_bt / agv_protocol — 블루투스
| 모듈 | 클래스 / 함수 | 책임 |
|------|--------------|------|
| `agv_protocol/messages.py` | `parse_command(line)`, `format_status(...)`, `format_event`, `format_error` | 채널 1 텍스트 메시지 (차량·노트북 공용) |
| `agv_protocol/framing.py` | `pack_frame(jpeg)`, `FrameReader` | 채널 2 길이+JPEG 프레임 |
| `rfcomm_server.py` | `RfcommServer` | 채널별 소켓 수락·연결마다 수신 스레드(새 연결이 이전 연결 대체), `is_alive`, TCP 모드(PC 시험) |
| `bridge_core.py` | `BridgeCore` | 명령 검증·허용 상태·하트비트 판단·DEV 전달 (ROS 없음, standalone 모드) |
| `sysinfo.py` | `SysMonitor` | Pi 온도·CPU·메모리 (DEV SYS) |
| `video_streamer.py` | `VideoStreamer` | 최신 프레임만 유지, 5fps JPEG 인코딩·전송 |
| `bt_bridge_node.py` | `BtBridgeNode` | 명령 → `/mission/command`·`/safety/estop`, `/mission/status` → STATUS 전송 |

### laptop_app — 노트북 앱
| 모듈 | 책임 |
|------|------|
| `main.py` | 앱 시작, 설정 읽기 |
| `bt_client.py` | 채널 1 연결·송수신 스레드, PING 0.5초, `agv_protocol` 사용 |
| `video_receiver.py` | 채널 2 수신 → `FrameReader` → 이미지 큐 |
| `ui/main_window.py` | 연결·작업 입력·비상 정지·상태 표시 (tkinter) |
| `ui/video_view.py` | 영상 표시 + 상태 문구 겹쳐 그리기 |
| `ui/manual_pad.py` | 방향 버튼·W/A/S/D → `MOVE` 0.3초 반복 |

---

## 5. 실행 구성

### 실행 방식
- 노드별 실행 파일(`entry_points`) → launch 파일로 묶어서 실행
- 영상 처리 노드는 처리량이 많아 **노드마다 별도 프로세스**로 실행(launch 기본 동작) → 한 노드가 느려져도 모터·안전 노드에 영향 없음
- 노드 안에서 오래 걸리는 작업(서비스 처리, BT 소켓)은 별도 스레드 또는 `MultiThreadedExecutor` + 콜백 그룹

### QoS
| 데이터 | 설정 | 이유 |
|--------|------|------|
| 카메라 영상 | BEST_EFFORT, 깊이 1 | 최신 프레임만 의미 있음 |
| `/line/state` | BEST_EFFORT, 깊이 1 | 오래된 오프셋은 버림 |
| `/cmd_vel` | RELIABLE, 깊이 1 | 최신 명령 |
| `/mission/command`, `/safety/estop`, `/led/state`, `/pantilt/pose` | RELIABLE, 깊이 10 | 명령 유실 방지 |
| `/mission/status` | RELIABLE, 깊이 1 | 최신 상태 |

### launch 파일 (테스트 트랙 단계와 대응)
| 파일 | 실행 노드 | 용도 |
|------|-----------|------|
| `s0_hw.launch.py` | camera, pantilt, motor, ultrasonic, led, bt_bridge | S0 기초 동작 |
| `s1_line.launch.py` | camera, motor, line, drive | S1~S3 라인·코너·정지선 |
| `s4_marker.launch.py` | camera, pantilt, marker | S4 마커 판별 |
| `s5_branch.launch.py` | + motor, line, drive, mission(분기 시험 모드) | S5 분기 |
| `s6_classify.launch.py` | camera, pantilt, classifier | S6·S7 인식, 학습 데이터 수집 |
| `agv.launch.py` | 전체 | S8~S10, 실제 운용 |

### 파라미터 파일 `config/params.yaml` (구조)
```yaml
motor_node:      { ros__parameters: { duty_max: 0.75, pwm_hz: 1000, wheel_map: [...], invert: [...], use_fake_hw: false } }
pantilt_node:    { ros__parameters: { pan_channel: 0, tilt_channel: 1, poses: { LINE: [70, 130], JUNCTION: [65, 165], SIGN: [65, 180] } } }
line_node:       { ros__parameters: { roi_ratio: 0.33, stop_line_width_ratio: [2.0, 5.0], confirm_frames: 3 } }
marker_node:     { ros__parameters: { hsv: { red: [...], green: [...], blue: [...], yellow: [...] }, min_area: 200, vote_frames: 10 } }
classifier_node: { ros__parameters: { cargo_model: models/cargo.tflite, sign_model: models/sign.tflite, min_score: 0.8 } }
ultrasonic_node: { ros__parameters: { rate_hz: 15, median_window: 5 } }
mission_node:    { ros__parameters: { load_distance: 0.10, load_hold_s: 3.0, obstacle_distance: 0.15, obstacle_clear_s: 1.0, cargo_retry: 2 } }
bt_bridge_node:  { ros__parameters: { heartbeat_timeout_s: 1.5, video_fps: 5, video_size: [320, 240], jpeg_quality: 50 } }
```
- 숫자는 초기값, 테스트 트랙 단계에서 보정 (기능 설계서 10절)

---

## 6. 의존성

| 위치 | 패키지 | 설치 |
|------|--------|------|
| Pi (Ubuntu 24.04) | ROS 2 Jazzy (`ros-jazzy-ros-base`), `cv_bridge`, `camera_ros` | apt (camera_ros는 바이너리가 없으면 소스 빌드) |
| Pi | libcamera (Raspberry Pi용) | 3단계에서 설치 방법 확정 — Camera Module 3 지원 필요 |
| Pi | `python3-opencv`, `python3-numpy`, `python3-lgpio`, `python3-smbus2` | apt |
| Pi | `ai-edge-litert` (TFLite 추론) | pip — Python 3.12 휠 지원 여부 3단계 확인 |
| 노트북 | Python 3.10+, `opencv-python`, `numpy`, `Pillow`, tkinter(표준), `agv_protocol` | pip (`pip install -e ros2_ws/src/agv_protocol`) |
| 노트북 | Teachable Machine (웹) | 브라우저 |

---

## 7. 테스트 전략

| 수준 | 대상 | 어디서 | 방법 |
|------|------|--------|------|
| 단위 | ② 알고리즘 (LineDetector, MarkerDetector, PidController, branch_selector, MissionStateMachine, agv_protocol) | 노트북·Pi | `pytest`. 영상 알고리즘은 테스트 트랙에서 찍은 **사진 묶음**으로 검사 |
| 시나리오 | MissionStateMachine 1사이클 | 노트북·Pi | 이벤트 순서를 넣어 기대 동작 목록 비교 (S10 흐름을 코드로 재현) |
| 노드 | 각 노드 + 가짜 드라이버 | Pi | `use_fake_hw: true`로 launch, `ros2 topic echo`로 확인 |
| 하드웨어 | 실제 드라이버 | Pi + 차량 | `04_unit/<기능>/` 점검 스크립트, 결과는 `test.md` |
| 통합 | 테스트 트랙 S0~S10 | 트랙 | 통과 기준은 [테스트 트랙](../track/test_track.md) |

---

## 8. 개발 흐름

```
노트북(Windows)                          Raspberry Pi 5 (Ubuntu)
──────────────                          ──────────────────────
② 알고리즘 작성 + pytest                  git pull
노트북 앱 작성                     ──►     colcon build --symlink-install
문서 작성 · 커밋 · push (GitHub)           launch로 단계별 실행 · 하드웨어 시험
                                         사진·로그 수집 → 노트북으로 가져와 알고리즘 개선
```
- 코드 공유는 GitHub 레포 하나로 (Pi에서는 읽기 위주, 수정은 노트북에서)
- Pi에서 찍은 시험용 사진은 용량이 크므로 GitHub에 올리지 않음 (`.gitignore`)

## 9. 코딩 규칙
- Python 3.12 기준, PEP 8, 타입 힌트 사용
- 노드 로그는 `self.get_logger()`, 노드 밖 모듈은 `logging`
- 파라미터 이름은 `snake_case`, 단위는 이름이나 주석에 표시 (`load_distance` = m, `_s` = 초)
- 상태·이벤트·동작 이름은 기능 설계서·블루투스 규약과 같은 대문자 문자열 사용
- 하드웨어 접근은 반드시 드라이버 클래스를 통해서만 (노드에서 lgpio 직접 호출 금지)

## 10. 구현 순서 (04_unit 순서와 대응)
> 팀원별 병렬 진행·주차 배치·관문은 [개발 순서 및 절차](../development_plan.md)에서 다룬다.

| 순서 | 구현 | 검증 |
|:---:|------|:---:|
| 1 | `agv_interfaces`, `agv_protocol`, `agv_bringup` 골격 | 빌드 |
| 2 | 드라이버 4종(실제·가짜) + hw 노드 4개, camera_ros 설정 | S0 |
| 3 | `LineDetector` + `line_node`, `PidController` + `DriveController` + `drive_node` | S1~S3, S8 |
| 4 | `MarkerDetector` + `marker_node`, `branch_selector` | S4, S5 |
| 5 | 노트북 앱 + `agv_bt` (제어 채널 먼저, 영상 나중) | S0 |
| 6 | `TfliteClassifier` + `classifier_node` (데이터 수집 → 학습) | S6, S7 |
| 7 | `MissionStateMachine` + `mission_node` | 시나리오 테스트 → S9, S10 |

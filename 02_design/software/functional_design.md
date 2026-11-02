# 기능 설계서 — 차량 소프트웨어 (ROS 2)

> 대상 하드웨어: PiCar-R5 키트(Raspberry Pi 5, DRV8835 × 2, PCA9685, TT 모터 4개, 팬틸트) + Camera Module 3 Wide + 브레드보드(초음파, LED 2개)
> 소프트웨어: Ubuntu 24.04 LTS · ROS 2 Jazzy · Python(rclpy) / 노트북 Python 앱
> 관련 문서: [핀맵](../hardware/pinmap.md) · [배선도](../hardware/wiring.md) · [테스트 트랙](../track/test_track.md) · [블루투스 규약](../communication/bluetooth_protocol.md) · [AI 클래스](../ai/tm_classes.md)

---

## 1. 기능 ↔ 하드웨어 대응

| # | 기능 | 하드웨어 | 담당 노드 | 검증 단계 |
|---|------|----------|-----------|:---:|
| F1 | 영상 획득 | Camera Module 3 Wide (IMX708, CSI) | `camera_ros` (기존 패키지) | S0 |
| F2 | 카메라 각도 전환 | SG90 × 2 ← PCA9685 (I2C) | `pantilt_node` | S0, S6 |
| F3 | 모터 구동 | TT 모터 4개 ← DRV8835 × 2 (GPIO 18~25) | `motor_node` | S0 |
| F4 | 라인 추종·이벤트 검출 | 카메라 | `line_node` + `drive_node` | S1~S3 |
| F5 | 직각 회전·유턴 | 모터 + 카메라 | `drive_node` | S2, S8 |
| F6 | 색 마커 판별 | 카메라 | `marker_node` | S4, S5 |
| F7 | 화물·표지판 인식 (AI) | 카메라 | `classifier_node` | S6, S7 |
| F8 | 거리 측정 (적재·장애물) | HC-SR04 (GPIO 5·6) | `ultrasonic_node` | S0, S7, S9 |
| F9 | 적재 상태 표시 | LED 빨강·초록 (GPIO 16·26) | `led_node` | S0, S7 |
| F10 | 블루투스 제어·상태 보고 | Pi 5 내장 BT | `bt_bridge_node` | S0, S10 |
| F11 | 블루투스 영상 전송 | Pi 5 내장 BT | `bt_bridge_node` | S0 |
| F12 | 물류 처리 (상태 머신) | — | `mission_node` | S10 |
| F13 | (옵션) LCD 작업 횟수 | I2C LCD (GPIO 2·3 공유) | `lcd_node` | — |

---

## 2. 시스템 구조

```
 ┌──────────────────────── Raspberry Pi 5 (ROS 2 Jazzy) ────────────────────────┐
 │                                                                              │
 │  camera_ros ──/camera/image_raw──┬──► line_node ──/line/state, /stop_line/state──► drive_node  │
 │                                  ├──► marker_node ─/marker/state─► (drive_node)        │
 │                                  ├──► classifier_node (서비스)      │  │      │
 │                                  └──► bt_bridge_node (영상)     /cmd_vel│      │
 │                                                                    ▼  │      │
 │  ultrasonic_node ──/ultrasonic/range──► mission_node ◄────────► motor_node   │
 │                                         │  ▲   │                             │
 │  pantilt_node ◄──/pantilt/pose──────────┘  │   └──/led/state──► led_node     │
 │                                            │                                 │
 │  bt_bridge_node ◄──/mission/status─────────┘                                 │
 │       │  └──/mission/command, /safety/estop──►                               │
 └───────┼──────────────────────────────────────────────────────────────────────┘
         │ Bluetooth RFCOMM (채널 1: 제어 / 채널 2: 영상)
 ┌───────┴──────── 노트북 Python 앱 ────────┐
 │  작업 횟수 입력 · 비상 정지 · 수동 제어 · 상태 표시 · 영상 표시 │
 └──────────────────────────────────────────┘
```

- `mission_node`가 전체 흐름(상태 머신)을 결정하고, `drive_node`는 "라인 따라가기·회전" 같은 주행 동작만 수행
- 영상 처리: 주행 중 `line_node`(라인·정지선)·`marker_node`가 프레임마다, `classifier_node`는 서비스 호출 시에만 처리

---

## 3. ROS 2 패키지 구성 (`ros2_ws/src/`)

> 모듈·클래스·계층 구조는 [소프트웨어 구조 설계](software_architecture.md)

| 패키지 | 빌드 | 내용 |
|--------|------|------|
| `agv_interfaces` | ament_cmake | 사용자 정의 메시지·서비스 (`.msg`, `.srv`) — 인터페이스 패키지는 CMake 필수 |
| `agv_hw` | ament_python | `motor_node`, `pantilt_node`, `ultrasonic_node`, `led_node` (lgpio, smbus2) |
| `agv_vision` | ament_python | `line_node`, `marker_node`, `classifier_node` (OpenCV, LiteRT) |
| `agv_control` | ament_python | `drive_node`, `mission_node` |
| `agv_bt` | ament_python | `bt_bridge_node` (Python socket RFCOMM) |
| `agv_protocol` | ament_python | 블루투스 메시지 만들기·해석 (순수 Python, 노트북 앱도 같은 코드 사용) |
| `agv_bringup` | ament_python | launch 파일, 파라미터(`params.yaml`), 모델 파일 |

---

## 4. 토픽 · 서비스 · 액션

### 토픽
| 이름 | 타입 | 발행 → 구독 | 주기 | 내용 |
|------|------|-------------|------|------|
| `/camera/image_raw` | sensor_msgs/Image | camera_ros → vision, bt | 15~30Hz | 카메라 영상 |
| `/line/state` | agv_interfaces/LineState | line_node → drive_node, marker_node, bt_bridge | 영상 주기 | 라인 오프셋, 이벤트 |
| `/stop_line/state` | agv_interfaces/StopLineState | line_node → drive_node, bt_bridge | 영상 주기 | 정지선 판정·확정·기울기 |
| `/marker/state` | agv_interfaces/MarkerState | marker_node → drive_node, bt_bridge | 영상 주기 | 보이는 색 스티커의 색·좌우(라인 기준)·화면 높이 |
| `/cmd_vel` | geometry_msgs/Twist | drive_node·mission(수동) → motor_node | 20Hz | 선속도 x, 각속도 z |
| `/ultrasonic/range` | sensor_msgs/Range | ultrasonic_node → mission_node | 15Hz | 거리(m), 필터 적용 |
| `/pantilt/pose` | std_msgs/String | mission_node → pantilt_node | 이벤트 | `LINE` / `JUNCTION` / `SIGN` |
| `/pantilt/ready` | std_msgs/String | pantilt_node → mission_node | 이벤트 | 자세 변경 후 안정화 완료 (자세 이름) |
| `/led/state` | std_msgs/String | mission_node → led_node | 이벤트 | `OFF` / `RED` / `GREEN` / `ERROR` |
| `/mission/command` | agv_interfaces/Command | bt_bridge → mission_node | 이벤트 | START n, STOP, RESUME, MANUAL, MOVE |
| `/mission/status` | agv_interfaces/Status | mission_node → bt_bridge | 2Hz + 변화 시 | 상태, 작업 횟수, 화물, 목적지 |
| `/safety/estop` | std_msgs/Bool | bt_bridge·mission → motor_node | 이벤트 | 비상 정지 래치 |
| `/dev/wheels` | std_msgs/Float32MultiArray | bt_bridge → motor_node | 0.3초 반복 | 개발자 탭: L1~L4 듀티 (-1~1), 워치독 적용 |
| `/drive/speed` | std_msgs/Float32 | bt_bridge → drive_node | 이벤트 | 개발자 탭 "주행 속도": 라인 주행 기본 선속도 (0.2~1.0), 주행 중에도 적용 |
| `/drive/tune` | std_msgs/String | bt_bridge → drive_node | 이벤트 | 개발자 탭 "회전 세기"(`TURN 0.6`)·"라인 기준"(`CENTER -0.1`)·"회전 전 전진"(`FORWARD 0.6`)·"제자리 회전"(`SPIN 0.6`), 주행 중에도 적용 |
| `/drive/target` | std_msgs/String | bt_bridge(시험)·mission_node(예정) → drive_node | 이벤트 | 목적지 색 `red`/`green`/`blue`/`yellow`, `NONE`이면 마커를 보지 않음 |
| `/line/tune` | std_msgs/String | bt_bridge → line_node | 이벤트 | 개발자 탭 "확인 프레임"(`CONFIRM 3`)·"라인 끝 프레임"(`LINE_END 10`), 실행 중 적용 |
| `/dev/pantilt_angle` | std_msgs/Float32MultiArray | bt_bridge → pantilt_node | 이벤트 | 개발자 탭: [팬, 틸트] 각도 |

### 서비스 / 액션
| 이름 | 종류 | 제공 | 요청 → 응답 |
|------|------|------|-------------|
| `/drive` | 액션 `agv_interfaces/Drive` | drive_node | 동작(`FOLLOW_TO_EVENT`, `TURN_LEFT`, `TURN_RIGHT`, `STRAIGHT`, `UTURN`, `APPROACH`, `STOP`) → 결과 이벤트·성공 여부 |
| `/vision/classify` | 서비스 `agv_interfaces/Classify` | classifier_node | 모드(`CARGO` / `SIGN`) → 라벨, 신뢰도 |

### 사용자 정의 메시지 (agv_interfaces)
```
# LineState.msg
float32 offset          # 라인 중심의 좌우 오차 (-1.0 ~ 1.0)
bool    line_found
string  event           # NONE / STOP_LINE / JUNCTION_T / JUNCTION_LEFT / JUNCTION_RIGHT
                        # CORNER_LEFT / CORNER_RIGHT / LINE_END
string  raw_event       # 이번 프레임 판정 (연속 프레임 확인 전) — 검출 값 조정용
float32 angle           # 라인 각도(도) — 90 = 똑바로, 90 미만 = 위쪽이 오른쪽으로 기움
bool    angle_valid

# StopLineState.msg
bool    detected        # 이번 프레임 판정 — drive_node는 처음 나오면 일단 정지
bool    confirmed       # 확인 프레임만큼 쌓여 확정
float32 angle           # 정지선 기울기(도), 0 = 수평, + = 오른쪽이 위

# MarkerState.msg — 세 배열은 같은 길이
string[]  colors         # red / yellow / green / blue
string[]  sides          # left / right (라인 기준)
float32[] y              # 화면 높이 — 클수록 차에 가까움

# Status.msg
string state            # 상태 머신 상태 이름
uint8  done             # 완료한 작업 수
uint8  total            # 입력한 작업 수
string cargo            # NONE / CIRCLE / SQUARE
string target           # WORK / DROP_A / DROP_B / PARK
string message          # 오류·알림 문구

# Command.msg  (bt_bridge → mission_node)
string kind             # START / STOP / RESUME / MANUAL_ON / MANUAL_OFF / MOVE
uint8  total            # START일 때 작업 횟수
float32 linear          # MOVE일 때 선속도 (-1.0 ~ 1.0)
float32 angular         # MOVE일 때 회전 (-1.0 ~ 1.0)

# Drive.action
string command          # FOLLOW_TO_EVENT / TURN_LEFT / TURN_RIGHT / STRAIGHT / UTURN / APPROACH / STOP
---
bool   success
string result           # 끝난 이유: STOP_LINE / JUNCTION_* / CORNER_* / TURN_DONE / UTURN_DONE / STOPPED / E_TURN / E_LINE
---
float32 offset          # 진행 중 피드백: 현재 라인 오프셋

# Classify.srv
string mode             # CARGO / SIGN
---
string label            # 확정 라벨, 미확정이면 빈 문자열
float32 score
```

---

## 5. 상태 머신 (`mission_node`)

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> UTURN : START n (목적지=작업장, 대기장소 표지판을 보고 서 있으므로 먼저 유턴)
    NAVIGATE --> CHECK_SIGN : 정지선 도착
    CHECK_SIGN --> APPROACH : 작업장 표지판
    CHECK_SIGN --> UNLOAD : 하차 A/B 표지판
    CHECK_SIGN --> PARKED : 대기장소 표지판
    APPROACH --> LOAD : 적재 거리 도달
    LOAD --> UTURN : 화물 인식 (목적지=하차 A/B)
    UNLOAD --> UTURN : 하차 완료 (목적지=작업장 또는 대기장소)
    UTURN --> NAVIGATE
    PARKED --> IDLE : 작업 종료 보고

    NAVIGATE --> PAUSED_OBSTACLE : 장애물
    PAUSED_OBSTACLE --> NAVIGATE : 1초간 장애물 없음
    NAVIGATE --> ERROR : 마커 판단 불가 / 라인 이탈
    CHECK_SIGN --> ERROR : 표지판 불일치
    LOAD --> ERROR : 화물 인식 실패 (재시도 초과)
    ERROR --> NAVIGATE : RESUME
```

- **공통 상태 (모든 상태에서 진입)**: `ESTOP`(BT 비상 정지·연결 끊김 → 모터 정지, RESUME 시 이전 상태로), `MANUAL`(BT 수동 제어, 해제 시 IDLE)
- `NAVIGATE`는 목적지(`target`)를 가지고 정지선에 도착할 때까지 라인 추종 + 분기점 판단을 반복하는 상위 상태 (구현에서는 분기점 처리를 `AT_JUNCTION` 상태로 분리 — [클래스 설계](class_design.md) 3.2)

### 상태별 동작
| 상태 | 진입 시 동작 | 종료 조건 | 다음 |
|------|-------------|-----------|------|
| IDLE | 모터 정지, LED OFF, 팬틸트 LINE | START n 수신 | target=WORK → UTURN (출발 위치 = 대기장소 정지선, 표지판 방향) |
| NAVIGATE | 팬틸트 LINE, `/drive FOLLOW_TO_EVENT` | 이벤트 수신 | 아래 6절 |
| CHECK_SIGN | 정지, 팬틸트 SIGN, `/vision/classify SIGN` | 표지판 확인 | 표지판 = 기대 목적지 → 해당 상태 / 불일치 → ERROR |
| APPROACH | 팬틸트 LINE, `/drive APPROACH`(서행, **라인 이벤트 무시** — 방금 멈춘 작업장 정지선 위를 지나가므로) | 초음파 ≤ 적재 거리 | LOAD |
| LOAD | 정지, 3초 동안 거리 유지 확인 + 팬틸트 SIGN, `/vision/classify CARGO` | 3초 유지 + 화물 확정 | LED RED/GREEN, target = DROP_A/B, **skip_stop_lines = 1**(돌아 나가며 작업장 정지선을 다시 지남) → UTURN |
| UNLOAD | 정지, LED OFF, 2초 대기(하차 연출), done += 1 | 대기 끝 | target = (done < total ? WORK : PARK) → UTURN |
| UTURN | 팬틸트 LINE, `/drive UTURN` | 라인 재인식 | NAVIGATE |
| PARKED | 정지, 완료 보고(BT) | 보고 완료 | IDLE |
| PAUSED_OBSTACLE | 정지, 상태 보고 | 1초 연속 장애물 없음 | 이전 동작 재개 |
| ERROR | 정지, LED ERROR(깜빡임), BT로 오류 코드 전송 | RESUME | 이전 상태 재시도 |

---

## 6. 주행 · 분기점 판단

### NAVIGATE 처리 흐름
```
FOLLOW_TO_EVENT ─► 이벤트?
   ├ CORNER_LEFT / CORNER_RIGHT ─► 정지 → 같은 방향 90° 회전 → FOLLOW 재개
   ├ 목적지 색 스티커 ─► 감속 → 정지·좌우 확정 → 왼쪽만 좌회전 / 오른쪽만 우회전 / 양쪽 직진 → FOLLOW 재개 (drive_node가 처리, 2026-11-02)
   ├ STOP_LINE ─► skip_stop_lines > 0 이면 1 줄이고 통과 / 아니면 정지 → CHECK_SIGN
   └ LINE_END (정지선 없이 끝) / 라인 이탈 ─► ERROR
```

### 이벤트 중복 검출 방지
| 상황 | 문제 | 규칙 |
|------|------|------|
| 작업장에서 적재 후 유턴 | 이미지판과 분기점 사이의 **작업장 정지선**을 다시 지나며 CHECK_SIGN에 잘못 들어감 | LOAD에서 `skip_stop_lines = 1` → 다음 정지선 1개 통과 |
| 작업장 정지선 → 이미지판 접근 | 멈춘 정지선 위를 바로 지나감 | APPROACH 중에는 라인 이벤트 무시 |
| 분기점에서 직진·회전 직후 | 같은 분기점의 가로 라인을 다시 이벤트로 검출 | `drive_node`는 STRAIGHT·TURN 완료 후 라인 이벤트가 사라질 때까지(최소 0.5초) 새 이벤트 무시 |
| 하차 지점·대기장소 유턴 | 정지선이 유턴 후 차량 뒤쪽에 있음 | 규칙 불필요 (S8에서 확인) |

### 분기점 판단 (색 마커 규칙, 2026-11-02 결정)
목적지 색 스티커의 **라인 기준 위치**가 방향을 정한다. 스티커는 분기점에 들어오는 라인의 옆에 붙인다.

| 목적지 색이 보이는 곳 | 동작 |
|---|---|
| 왼쪽만 | 좌회전 |
| 오른쪽만 | 우회전 |
| 양쪽 | 직진 (양쪽이 한 프레임에 보이면 멈추지 않음) |
| 안 보임 | 라인을 따라 계속 주행 |

- 다른 색만 보여도 오류로 보지 않는다 — 분기점을 지난 직후에도 다른 방향용 스티커가 정상적으로 보이기 때문. **스티커가 모두 떨어지면 그 자리에서 알 수 없고** T자는 `LINE_END`로 멈추며, 직진 갈래가 있는 곳은 도착 후 표지판 확인으로 잡는다
- 직진 스티커 2장 중 1장만 떨어지면 회전으로 잘못 읽는다 → 라인을 못 찾으면 시간 제한으로 `E_LINE`. 제한 시간은 90°보다 조금 더 도는 시간으로 맞춘다 (길면 들어온 라인까지 돌아가 되돌아갈 수 있음)

## 7. 기능별 상세 설계

### F1 · 영상 획득 (`camera_ros`)
- 카메라: Raspberry Pi Camera Module 3 Wide (IMX708, 대각 약 120° 광각, 자동 초점)
- 출력 640×480 → 노드마다 필요한 크기로 축소 (라인 320×240, AI 224×224, BT 영상 320×240)
- **초점**: 주행 중 자동 초점이 흔들리면 라인 검출이 불안정 → 자세별 고정 초점(LINE·JUNCTION·SIGN 각각 렌즈 위치 값)으로 운용, S0·S6에서 값 결정
- **광각 왜곡**: 화면 가장자리가 휘어 보임 → 라인 추종은 화면 아래 중앙 관심 영역만 사용, 마커 위치 판단에 문제가 되면 왜곡 보정(카메라 보정 1회) 적용
- 광각 덕분에 분기점에서 좌·앞·우 갈래 마커를 한 화면에 담기 쉬움

### F2 · 팬틸트 (`pantilt_node`)
- PCA9685(I2C 0x40 기본) 채널 = 서보 헤더 번호(조립 시 확정), 50Hz
- 자세 3가지 (각도는 S0·S6에서 보정, `params.yaml`)
  | 자세 | 용도 | 틸트 |
  |---|---|---|
  | LINE | 주행 중 라인·정지선·분기점 검출 | 아래 |
  | JUNCTION | 분기점에서 갈래 마커 보기 | 약간 아래 (갈래 3개가 한 화면에) |
  | SIGN | 표지판·화물 인식 | 정면 |
- 자세 변경 후 0.3초 안정화 대기
- GPIO17: 제어 없이 서보 동작 확인(2026-10-26) → 필수 아님. 드라이버는 안전하게 시작 시 LOW 출력(OE라면 출력 활성, 아니면 영향 없음)

### F3 · 모터 (`motor_node`)
- 입력 `/cmd_vel`(x: 선속도, z: 각속도) → 좌우 속도 = x ∓ z·k → 같은 쪽 바퀴 2개는 같은 값
- lgpio 소프트웨어 PWM 1kHz, DIR 핀으로 방향, **듀티 상한 75%**(TT 모터 보호, 파라미터)
- 바퀴 위치·방향 보정표(`params.yaml`): L1~L4 → 좌전·좌후·우전·우후, 극성 반전 여부 (S0에서 확정)
- 안전: `/cmd_vel` 0.5초 이상 끊기면 정지(워치독), `/safety/estop` true면 래치 정지

### F4 · 라인 추종 (`line_node` + `drive_node`)
- 처리: 영상 320×240으로 축소 → 하단 1/3 관심 영역 → 검정 추출(HSV: 줄의 바닥 밝기(상위 10%) × 0.72 이하 + 채도 낮음, 색 마커 제외 — 2026-11-02 광택 테이프 반사로 고정 문턱이 라인을 잃어 변경) → 연결 성분
- 라인 고르기 (2026-10-30, 주변 어두운 물체 대책): 줄마다 **폭 ≤ max_line_px(45px)인 가는 구간만** 라인 후보 — 두꺼운 물체는 붙어 있든 떨어져 있든 제외. 덩어리 점수 = 가는 구간 면적 × 가운데 가중치(양끝 0.3) × 직전 위치 가중치 × 화면 맨 아래에 닿으면 1.5배. 이벤트 판정은 **고른 라인과 이어진 영역만** 사용 — 가중치를 이벤트에 쓰면 화면 끝까지 뻗는 코너 라인을 놓치므로
- 오프셋: 고른 라인의 가까운 띠(관심 영역 아래 1/3)에서 아래 줄부터 위로, 줄마다 바로 아래 줄 위치에 가까운 가는 구간 중심의 평균 → -1.0 ~ 1.0. 라인을 못 찾으면 마지막 값 유지 + LINE_END
- 이벤트 검출 (위쪽 행 기준)
  | 이벤트 | 판단 |
  |---|---|
  | STOP_LINE | **`StopLineDetector`가 따로 판정** (같은 `line_node`) — `/stop_line/state`(아래 "정지선") |
  | ~~JUNCTION_T / LEFT / RIGHT~~ | **판정 안 함 — 분기점은 색 마커 인식이 맡음 (2026-10-30 결정)**. T자·옆 갈래 모양은 NONE, 마커를 놓쳐 T자에서 라인이 끊기면 LINE_END로 정지 |
  | CORNER_LEFT / RIGHT | 앞쪽 라인 없음 + 한쪽으로만 가로 라인 (폭이 라인 폭의 2배 이상 — 곧은 라인이 화면 끝에 있는 경우 제외) |
  | LINE_END | 관심 영역에 라인 없음 — 가까운 띠에서만 놓치고 직전 위치 근처 위쪽(중간·먼 띠)에 라인이 보이면 "잠깐 놓침"(NONE, 감속, 라인 끝 프레임의 2배까지), **10프레임 연속**이어야 확정 (2026-10-30: 3프레임이면 흔들림으로 잠깐 놓쳐도 멈춤) |
- **정지선 (`StopLineDetector`, 2026-11-02 세로 열 방식, 노드는 CPU를 줄이려고 `line_node`에 합침):** 가로 줄 방식은 기운 정지선을 놓침(실제 사진 +5° 대부분, ±15° 전부). 같은 검정 추출을 쓰고, 라인과 이어진 덩어리에서 **열마다** 가장 긴 세로 구간을 본다
  - 관심 영역은 라인(하단 1/3)보다 넓은 **하단 1/2** — 일찍 잡아야 지연·관성으로 지나쳐도 영역 안에서 멈춤
  - 관심 영역 위·아래에 닿는 열 묶음 = 라인 (정지선이 가까운 띠에 들어오면 라인 오프셋이 끌려가므로 위치는 이것으로 판단, 오프셋은 덩어리 고르는 데만 사용)
  - 정지선 열 = 위·아래에 닿지 않고 두께 ≥ 라인 폭 × 0.65 (위에 닿으면 아직 들어오는 중, 아래에 닿으면 곡선의 대각선 라인일 수 있음). 단 판정 후 20프레임 동안은 아래로 나가는 구간(두께 ≤ 라인 폭 × 2.5)도 인정 — 지나쳐 멈춘 뒤 판정이 사라져 재출발하는 것 방지. 3열까지 끊겨도 이어진 것으로 봄(테이프 반사)
  - 판정 = 정지선 열이 라인 왼쪽·오른쪽 모두 라인 폭 × 0.5 이상, 전체 폭 라인 폭의 3~12배, 화면 양끝 동시 도달 아님(T자 → 마커 인식), 기울기 ≤ 35°
  - `drive_node`·`bt_bridge`가 `/line/state`에 합쳐 씀(0.3초보다 오래된 판정은 버림) — 앱 표시·주행 규칙은 그대로
- 라인 각도 (2026-10-30, 차량이 비스듬하면 라인이 대각선으로 보이는 문제): 관심 영역 맨 아래부터 위로 줄마다 라인 중심을 따라가 직선을 맞춰 각도 계산(90° = 똑바로, 90° 미만 = 위쪽이 오른쪽). 끝점 2개 대신 직선 맞춤 — 끝부분 흔들림에 덜 민감
- `drive_node` PID: 각속도 z = −turn_scale·(Kp·e + Kd·Δe + Kh·각도 오차/45°) (Ki는 필요 시), e = 오프셋 − center_offset(카메라가 차체 중심에서 비켜 달린 만큼, 2026-10-30 −0.10), 각도 오차는 85~95° 밖으로 벗어난 만큼만(`heading_deadband_deg` 5°). 선속도는 오차가 크면 감속. **정지선·코너는 판정 첫 프레임에 일단 정지하고 멈춘 채 확인 — 확정되면 끝(한 프레임 빠짐 허용), 판정이 5프레임 연속 사라지면 다시 출발** (2026-11-02: 달리면서 확인 프레임을 채우면 지나침). **LINE_END는 정지 후 3번(0.5초씩) 다시 봐서 라인이 3프레임 연속 보이면 자동 재출발, 끝까지 없으면 종료** (2026-11-02: 주행 중 잠깐 놓친 경우)
- 같은 이벤트가 연속 3프레임 이상일 때만 확정 (노이즈 방지)

### F5 · 직각 회전·유턴 (`drive_node`)
- TURN_LEFT/RIGHT: 정지선/분기점 위치에서 차체 반 길이 전진 → 제자리 회전(좌우 반대 방향) → 회전 중 라인이 화면 중앙에 들어오면 정지 → FOLLOW
- UTURN (2026-11-02 구현): 전진 없이 왼쪽으로 제자리 회전 → 기존 라인이 벗어난 뒤 라인이 기준 위치 ± 0.4, 각도 90 ± 35°로 3프레임 연속 보이면 정지·`UTURN_DONE`. 각도는 재지 않고, 시작 후 1.5초(`uturn_min_s`)는 라인으로 인정하지 않음. 8초 안에 못 찾으면 `E_TURN`. 회전 세기는 앱 "제자리 회전"과 같은 값. 시험은 개발자 탭 "U턴" 버튼 (`DEV DRIVE UTURN`)
- 안전 시간 제한: 마커 회전 4초 / 유턴 8초 (실제 회전 속도를 보고 줄임)
- 회전 속도·전진 거리는 S2·S8에서 보정

### F6 · 색 마커 판별 (`marker_node`)
- 마커 = 지름 약 2cm 원형 스티커. 카메라 자세는 LINE 그대로 (JUNCTION 자세로 바꾸지 않음)
- 처리 (`agv_vision/marker.py`): 320×240 → HSV → 색별 범위 → 열림(3×3) → 연결 성분 → 스티커 조건 (면적 ≥ 30, 면적 ÷ 외접 사각형 ≥ 0.5, 가로 폭 = 높이별 예상 지름의 0.5~1.6배, 세로 ≤ 2배)
- 좌우는 **라인 기준** (카메라가 차체 중심에서 비켜 있음). 라인이 기울면 라인 각도로 스티커 높이의 라인 위치를 계산해 비교
- HSV 범위 (2026-11-02 책상 위 실제 스티커 사진, 트랙에서 다시 확인): 빨강 H168~10·S≥125·V≥125, 노랑 H24~38·S≥120·V≥120, 초록 H60~85·S≥90·V≥70 (가장 약함), 파랑 H100~116·S≥120·V≥90 → `params.yaml`
- 출력 `/marker/state`: 색, 좌우, 화면 높이 — 높이는 거리 대신 쓰고, 앱 "MARKER" 측정값으로 확인
- 판단은 `drive_node` (`DriveController`, 목적지 색이 정해졌을 때만):
  1. 주행 중 목적지 색이 y ≥ 60에 보이면 감속, y ≥ 120까지 오면 정지 — 높이로 정해서 속도와 상관없이 비슷한 자리에 섬
  2. 0.3초 뒤부터 5프레임을 보고 과반으로 좌우 확정 (y ≥ 90만 — 건너편 스티커 제외). 사라지면 오판정으로 보고 다시 출발
  3. 회전: **전진** (회전 중심을 분기점 가까이, 카메라가 차축보다 앞을 보기 때문) → **제자리 회전** → 기존 라인이 벗어난 뒤 새 라인이 보이면 끝 → 라인 주행. 회전 각도는 재지 않는다 (엔코더 없음)
     - 기존 라인이 벗어남: 라인이 인정 범위 밖으로 2프레임, 또는 오프셋이 도는 쪽으로 0.25 이상 튐 (광각이라 기존 라인이 나가기 전에 새 라인이 들어옴)
     - 새 라인 인정: 기준 위치 ± 0.4, 각도 90 ± 35° (T자 가로선 제외), 3프레임 연속
     - 4초 안에 못 찾으면 정지·`E_LINE`
  4. 회전 뒤 1초는 정지선·코너·라인 끝 무시, 직진·회전 뒤 2초는 마커 무시 (같은 표시 재인식 방지)
- 전진 시간은 스티커 위치와 실제 속도에 맞춰 앱 "회전 전 전진"으로 조정

### F7 · 화물·표지판 인식 (`classifier_node`)
- Teachable Machine 이미지 모델 → TFLite 변환 → LiteRT(`ai-edge-litert`)로 추론, 입력 224×224
- 모델 2개: 모드 CARGO → `cargo.tflite`(`circle` / `square` / `background`), 모드 SIGN → `sign.tflite`(`sign_work` / `sign_drop_a` / `sign_drop_b` / `sign_park` / `background`)
- 5프레임 연속 추론 → 신뢰도 0.8 이상이 3회 이상 같은 라벨이면 확정, 아니면 실패
- 화물 인식 실패 시 2회 재시도 → 그래도 실패면 ERROR(BT 알림) — 클래스 상세는 [tm_classes.md](../ai/tm_classes.md)

### F8 · 초음파 (`ultrasonic_node`, 해석은 `mission_node`)
- 15Hz 측정(Trig 10µs 펄스 → Echo 펄스 폭), 이동 중앙값 필터 5개, 범위 밖(2cm 미만·200cm 초과)은 무시
- 해석은 상태에 따라 다름
  | 상태 | 해석 | 기준 (파라미터, S7·S9에서 보정) |
  |---|---|---|
  | APPROACH / LOAD | 적재 거리 | 거리 ≤ 10cm 를 3초 유지 (±2cm) |
  | NAVIGATE, UTURN | 장애물 | 거리 ≤ 15cm → PAUSED_OBSTACLE, 1초 연속 해제 시 재개 |
  | 기타 | 무시 | — |
- **배치 제약**: 정지선에 멈췄을 때 차량 앞~표지판(또는 이미지판) 거리가 장애물 거리보다 짧으면, 정지선에 닿기 전에 표지판을 장애물로 보고 멈춘다 → **표지판·이미지판은 정지선에서 장애물 거리 + 10cm 이상(초기 25cm) 떨어뜨린다**

### F9 · LED (`led_node`)
| 명령 | 빨강 GPIO16 | 초록 GPIO26 |
|---|---|---|
| OFF (빈 차) | 끔 | 끔 |
| RED (원 적재) | 켬 | 끔 |
| GREEN (사각형 적재) | 끔 | 켬 |
| ERROR | 0.5초 간격 둘 다 깜빡임 | 〃 |

### F10·F11 · 블루투스 (`bt_bridge_node`)
- RFCOMM 서버, **채널 1 제어 / 채널 2 영상** 분리 → 영상이 밀려도 제어 명령은 지연되지 않음
- 하트비트: 노트북이 0.5초마다 PING, 1.5초 동안 아무 수신이 없으면 연결 끊김 → `/safety/estop` true
  - 부팅 후 **첫 연결 전에는 감시하지 않음** (차량은 IDLE로 정지 상태라 안전, START는 연결 후에만 가능)
- 비상 정지 해제: `/safety/estop` true는 `motor_node`에 래치되고, RESUME 시 `mission_node`가 false를 발행해야만 풀린다
- 영상: `/camera/image_raw` → 320×240 JPEG(품질 50) → 초당 5프레임, 보낼 차례에 최신 프레임만 전송(밀린 프레임 버림)
- 메시지 형식은 [bluetooth_protocol.md](../communication/bluetooth_protocol.md)

---

## 8. 예외 처리 대응

| 상황 | 감지 | 대응 | 복귀 |
|------|------|------|------|
| 장애물 | 주행 중 초음파 ≤ 15cm | PAUSED_OBSTACLE (정지) | 1초간 장애물 없음 → 자동 재개 |
| 블루투스 끊김 | 1.5초 무수신 | ESTOP (모터 정지) | 재연결 + RESUME |
| 비상 정지 | BT `STOP` | ESTOP | RESUME |
| 화물 인식 실패 | 재시도 2회 실패 | ERROR + BT 알림 | 화물 정리 후 RESUME |
| 마커 판단 불가 | 목적지 색 갈래 ≠ 1개 | ERROR + BT 알림 | 마커 정리 후 RESUME |
| 표지판 불일치 (옵션) | 기대 목적지와 다른 표지판 | ERROR + BT 알림 | 수동으로 차량 이동 후 RESUME |
| 라인 이탈 (옵션) | LINE_END/라인 없음 1초 | 정지 → 좌우로 천천히 탐색(2초) → 실패 시 ERROR | RESUME |
| 회전 실패 | 회전 시간 제한 초과 | ERROR | RESUME |

---

## 9. 운용 규칙
| 항목 | 규칙 |
|------|------|
| 출발 위치 | 대기장소 정지선, **표지판을 바라보는 방향** (주차가 끝난 자세와 같음) → START 시 유턴부터 |
| 부팅 자동 실행 | Pi 부팅 시 `agv.launch.py`를 systemd 서비스로 자동 실행 (시연 때 모니터·키보드 없이 사용) — 3단계에서 설정 |
| 배터리 | 작업 전 키트 전압계로 7.4V 배터리 전압 확인, 기준(예: 7.0V) 미만이면 충전 후 운용 — 기준값은 S0에서 결정 |
| 이미지판 교체 | 차량이 하차 지점에 도착한 뒤 교체 (APPROACH 중 손이 들어가면 적재로 오판) |
| 종료 | 노트북 앱 연결 해제 → 전원 스위치 OFF (Pi는 `shutdown` 후 끄는 것이 안전) |

## 10. 파라미터 (`agv_bringup/config/params.yaml` 예정)
| 그룹 | 항목 | 초기값 | 보정 단계 |
|------|------|--------|:---:|
| motor | 듀티 상한, 바퀴 위치·극성, PWM 주파수 | 75%, —, 1kHz | S0 |
| drive | 주행 속도, PID 게인, 회전 속도, 회전 전 전진 거리 | — | S1, S2, S8 |
| line | 이진화 방식, 관심 영역, 정지선 폭 비율 | Otsu, 하단 1/3, 2~5배 | S1, S3 |
| marker | 4색 HSV 범위, 최소 면적, 다수결 프레임, 갈래 영역 경계(back 제외 영역 포함) | — , —, 10, — | S4, S5 |
| layout | 정지선~표지판·이미지판 거리 | 25cm 이상 | S6, S7 |
| pantilt | LINE·JUNCTION·SIGN 각도, 서보 채널 | — | S0, S5, S6 |
| ultrasonic | 적재 거리·유지 시간, 장애물 거리·해제 시간 | 10cm·3초, 15cm·1초 | S7, S9 |
| classifier | 모델 경로, 신뢰도 기준, 재시도 | —, 0.8, 2 | S6, S7 |
| bt | 하트비트 시간, 영상 해상도·fps·품질 | 1.5초, 320×240·5·50 | S0 |

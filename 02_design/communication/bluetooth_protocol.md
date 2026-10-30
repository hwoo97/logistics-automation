# 블루투스 통신 규약 — Pi 5 ↔ 노트북 앱

> 방식: Bluetooth Classic **RFCOMM**, Pi 5 = 서버(`bt_bridge_node`), 노트북 = 클라이언트(Python 앱)
> Python 표준 `socket` 모듈(`AF_BLUETOOTH`, `BTPROTO_RFCOMM`) 사용 — Ubuntu·Windows 모두 지원
> 관련: [기능 설계서](../software/functional_design.md) F10·F11

## 1. 채널 구성

| 채널 | 용도 | 방향 | 형식 |
|:---:|------|------|------|
| 1 | 제어·상태 | 양방향 | 텍스트 한 줄 = 메시지 1개 (UTF-8, 끝에 `\n`) |
| 2 | 영상 | Pi → 노트북 | 바이너리 프레임 (길이 + JPEG) |

- 제어와 영상을 **채널로 분리**해 영상이 밀려도 비상 정지 등 제어 명령이 늦어지지 않게 한다
- 페어링은 3단계 준비에서 한 번 수행 (Pi `bluetoothctl`, 노트북 OS 블루투스 설정)

## 2. 노트북 → Pi (채널 1)

| 메시지 | 예시 | 의미 | 허용 상태 |
|--------|------|------|-----------|
| `START <n>` | `START 3` | 작업 횟수 n(1~9)으로 작업 시작 | IDLE |
| `STOP` | `STOP` | **비상 정지** — 즉시 모터 정지 (최우선 처리) | 모든 상태 |
| `RESUME` | `RESUME` | 비상 정지·오류 후 재개 | ESTOP, ERROR |
| `MANUAL ON` / `MANUAL OFF` | `MANUAL ON` | 수동 제어 모드 켜기/끄기 | IDLE, ESTOP, ERROR |
| `MOVE <v> <w>` | `MOVE 0.5 -0.2` | 수동 이동: 선속도 v, 회전 w (각 -1.0~1.0, 0.3초마다 반복 전송, 끊기면 정지) | MANUAL |
| `PING` | `PING` | 연결 확인 (0.5초마다) | 모든 상태 |

## 3. Pi → 노트북 (채널 1)

| 메시지 | 예시 | 의미 |
|--------|------|------|
| `STATUS <state> <done>/<total> <cargo> <target>` | `STATUS NAVIGATE 1/3 CIRCLE DROP_A` | 현재 상태 (2Hz + 상태가 바뀔 때) |
| `EVENT <type> <내용>` | `EVENT LOADED CIRCLE` | 주요 사건: `STARTED`, `LOADED`, `UNLOADED`, `PARKED`, `OBSTACLE`, `RESUMED` |
| `ERROR <code> <내용>` | `ERROR E_MARKER red 갈래 0개` | 오류 발생 (아래 코드표) |
| `ACK <명령>` / `NAK <명령> <이유>` | `NAK START 작업 중` | 명령 수신 결과 (`DEV`는 3-1절) |
| `PONG` | `PONG` | PING 응답 |

### 오류 코드
| 코드 | 의미 |
|------|------|
| `E_CARGO` | 화물 인식 실패 (재시도 초과) |
| `E_MARKER` | 분기점 목적지 마커 판단 불가 |
| `E_SIGN` | 표지판이 기대 목적지와 다름 |
| `E_LINE` | 라인 이탈·재탐색 실패 |
| `E_TURN` | 회전 시간 제한 초과 |
| `E_LINK` | 블루투스 연결 끊김 (재연결 후 보고) |

## 3-1. 개발자 명령 (채널 1, `DEV`)

노트북 앱 **개발자 탭**에서 하드웨어를 직접 조작하고 내부 값을 보기 위한 명령. `bt_bridge_node`가 받아 해당 ROS 토픽으로 전달한다(하드웨어 직접 접근 없음). 작업 중 끼어들지 않도록 **허용 상태에서만** 받고, 그 외에는 `NAK DEV <하위명령> <이유>`.

**노트북 → Pi**
| 메시지 | 예시 | 의미 | 전달 토픽 | 허용 상태 |
|--------|------|------|-----------|-----------|
| `DEV LED <OFF\|RED\|GREEN\|ERROR>` | `DEV LED RED` | LED 표시 | `/led/state` | IDLE, MANUAL, ESTOP, ERROR |
| `DEV POSE <LINE\|JUNCTION\|SIGN>` | `DEV POSE SIGN` | 팬틸트 정해진 자세 | `/pantilt/pose` | IDLE, MANUAL, ESTOP, ERROR |
| `DEV SERVO <pan> <tilt>` | `DEV SERVO 90 60` | 팬·틸트 각도 직접 지정 (0~180°) | `/dev/pantilt_angle` | IDLE, MANUAL, ESTOP, ERROR |
| `DEV WHEEL <l1> <l2> <l3> <l4>` | `DEV WHEEL 0.3 0.3 0 0` | 바퀴 4개 개별 듀티 (-1.0~1.0, 0.3초마다 반복, 끊기면 정지) | `/dev/wheels` | IDLE, MANUAL |
| `DEV SPEED <v>` | `DEV SPEED 0.7` | 라인 주행 기본 속도 (0.2~1.0) — drive_node에 바로 적용, 주행 중에도 | `/drive/speed` | 모든 상태 |
| `DEV WATCH <RANGE\|LINE\|SYS\|ALL> <ON\|OFF>` | `DEV WATCH RANGE ON` | 측정값 보내기 켜기/끄기 (연결 끊기면 모두 꺼짐) | — | 모든 상태 |
| `DEV NODES` | `DEV NODES` | 실행 중인 ROS 노드 목록 요청 | — | 모든 상태 |

**Pi → 노트북** (켠 항목만, 5Hz 이하)
| 메시지 | 예시 | 의미 |
|--------|------|------|
| `DEV RANGE <m>` | `DEV RANGE 0.234` | 초음파 거리(m, 필터 적용) — `/ultrasonic/range` |
| `DEV LINE <offset> <event> [<raw_event> [<angle>]]` | `DEV LINE -0.12 NONE STOP_LINE 87.5` | 라인 오프셋(-1~1), 확정 이벤트, 이번 프레임 판정(확인 전, 생략 시 event와 같음), 라인 각도(도, 90 = 똑바로, 못 믿으면 `-`) — `/line/state` |
| `DEV SYS <temp_c> <cpu_pct> <mem_pct>` | `DEV SYS 52.1 37 41` | Pi CPU 온도·사용률·메모리 (1Hz) |
| `DEV NODES <이름,...>` | `DEV NODES camera,led_node,motor_node` | `DEV NODES` 응답 |

- `DEV` 명령의 수신 결과는 `ACK DEV <하위명령>` / `NAK DEV <하위명령> <이유>` (예: `NAK DEV WHEEL NAVIGATE 상태에서 불가`)
- `mission_node`가 아직 없을 때(단위 개발 단계) 상태는 `IDLE`로 본다
- `DEV WHEEL`은 비상 정지 래치 중에는 `motor_node`가 무시한다 (안전 우선)

## 4. 영상 프레임 (채널 2)

```
┌──────────────┬───────────────────────────┐
│ 길이 4바이트   │ JPEG 데이터 (길이만큼)        │
│ (big-endian) │ 320×240, 품질 50           │
└──────────────┴───────────────────────────┘
```
- 초당 5프레임 목표, 보낼 차례에 **가장 최근 프레임만** 전송 (밀린 프레임 버림)
- 노트북은 길이를 먼저 읽고 그만큼 받아 디코딩·표시
- 화면에 상태 문구(STATUS)를 겹쳐 표시하는 것은 노트북 앱에서 처리

## 5. 연결 관리 · 안전

- **하트비트**: 노트북은 0.5초마다 `PING`. Pi는 채널 1에서 **1.5초 동안 아무것도 받지 못하면 연결 끊김**으로 판단 → 즉시 비상 정지(ESTOP)
  - 부팅 후 첫 연결 전에는 감시하지 않는다 (차량은 IDLE·정지 상태, `START`는 연결 후에만 받을 수 있음)
- 재연결 후 차량은 ESTOP 상태 유지 → 사용자가 상태를 보고 `RESUME`을 보내야 재개
- `STOP`은 수신 즉시 `/safety/estop`으로 모터를 멈추고, 이후 상태 머신에 전달 (상태 머신 처리 지연과 무관)
- 수동 이동 `MOVE`는 0.3초마다 다시 보내야 하며, 0.5초 끊기면 모터 워치독이 정지시킴

## 6. 노트북 앱 화면
| 위치 | 영역 | 내용 |
|------|------|------|
| 상단 (공통) | 연결 | Pi 주소·연결/해제, 연결 상태, **비상 정지(항상 보임, 스페이스바)** |
| **사용자 탭** | 작업 | 작업 횟수 입력 → 시작, 재개 |
| | 상태 | 현재 상태, 완료/전체 작업 수, 적재 화물, 목적지, 최근 사건·오류 |
| | 영상 | 채널 2 영상 + 상태 문구 |
| | 수동 제어 | 수동 모드 켜기/끄기, 방향 버튼 또는 W/A/S/D |
| **개발자 탭** | 하드웨어 조작 | LED, 팬틸트 자세·각도, 바퀴 4개 개별 듀티 |
| | 측정값 | 초음파 거리 그래프, 라인 오프셋·이벤트, Pi 온도·CPU·메모리, ROS 노드 목록 |
| | 영상 | FPS·전송 속도 표시, 사진 저장 |
| | 콘솔 | 주고받은 모든 메시지, 명령 직접 입력, 로그 파일 저장 |

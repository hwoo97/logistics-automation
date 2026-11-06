# 소프트웨어 베이스라인 관리

> 목적: 실제 차량에서 **확인된 상태**를 기준선(베이스라인)으로 고정해, 문제가 생기면 그 시점과 비교하거나 되돌릴 수 있게 한다.
> 관련: [개발 순서 및 절차](development_plan.md) · [소프트웨어 구조](software/software_architecture.md) · [블루투스 규약](communication/bluetooth_protocol.md)
> 최종 갱신: 2026-11-02

---

## 1. 형상 항목 (베이스라인에 묶는 대상)

| 구분 | 항목 | 위치 | 식별 |
|---|---|---|---|
| 차량 코드 | ROS 2 패키지 7개 — `agv_interfaces`, `agv_protocol`, `agv_hw`, `agv_vision`, `agv_control`, `agv_bt`, `agv_bringup` | `ros2_ws/src/` | Git 커밋 + 패키지 버전(`package.xml`·`setup.py`) |
| 노트북 앱 | tkinter 앱, 가짜 차량 | `laptop_app/` | Git 커밋 (`agv_protocol`을 같이 씀) |
| 인터페이스 | 메시지·서비스·액션(`agv_interfaces`), 블루투스 메시지(`agv_protocol`) | 위 패키지 | 바뀌면 차량·앱을 **같은 베이스라인으로** 함께 갱신 |
| 설정 | 노드 파라미터, launch | `agv_bringup/config/params.yaml`, `agv_bringup/launch/` | Git 커밋 |
| AI 모델 | 표지판 모델 `sign.tflite`, `labels_sign.txt` | `agv_bringup/models/` | 파일 SHA-256을 베이스라인 기록에 적음 |
| 차량별 값 | 트랙 매핑(초기 설정 결과), 앱 조정값 | Pi `~/.agv/track_map.json`, PC `laptop_app/config.json` | Git 밖 — 트랙·차량마다 다름, 기록만 |
| 설계 문서 | 기능·구조·클래스·규약·트랙·하드웨어 | `01_planning/`, `02_design/` | Git 커밋 (코드와 같은 커밋에서 맞춤) |
| 실행 환경 | Ubuntu 24.04, ROS 2 Jazzy, Python 3.12, camera_ros, OpenCV, lgpio, LiteRT | Pi | 아래 4절 명령으로 기록 |

---

## 2. 이름 규칙

| 항목 | 규칙 | 예 |
|---|---|---|
| 베이스라인 ID | `BL-<번호>` — 실차에서 확인된 단계마다 1씩 | `BL-3` |
| Git 태그 | 주석 태그 `bl<번호>-<짧은 이름>` (태그 메시지에 확인 근거) | `bl3-marker-uturn` |
| 패키지 버전 | `0.<번호>.0` — 7개 패키지 같은 값. 베이스라인 안의 수정은 `0.<번호>.<수정>` | `0.4.0` |
| 관문 태그 | 개발 계획의 `g1`~`g4`는 베이스라인 태그와 별도로 유지 | `g2` |

- 태그는 **실차 확인이 끝난 커밋에만** 붙인다. 시험 전인 커밋은 "후보"로만 적는다
- 태그는 지우거나 옮기지 않는다 (강제 push 금지 규칙과 같음). 잘못 붙였으면 다음 번호로 새로 만든다

---

## 3. 베이스라인 목록

| ID | 날짜 | 커밋 | 태그 | 내용 | 확인 근거 | 상태 |
|---|---|---|---|---|---|---|
| BL-0 | 2026-10-26 | `cd021d6` | `bl0-design` | 설계 문서 일체(기능·구조·클래스·규약·트랙·하드웨어), 개발 계획 | 설계 검토 완료 (2단계) | 확정 — 태그 생성 대기 |
| BL-1 | 2026-10-29 | `80bc1b8` | `bl1-s0-hw` | S0 하드웨어 — 모터 4개·팬틸트·초음파·LED, 블루투스 브리지, 노트북 앱 | Pi colcon test 86개 통과, 실제 하드웨어 S0 (바퀴 배치·방향 확정) | 확정 — 태그 생성 대기 |
| BL-2 | 2026-10-30 | `00a7716` | `bl2-s1-line` | S1 라인 주행 — `line_node`, `drive_node`, 앱 출발·정지·조정 | 실차 라인 주행 (정지선은 이후 개선) | 확정 — 태그 생성 대기 |
| BL-3 | 2026-11-02 | `323dce1` | `bl3-marker-uturn` | 정지선(세로 열 방식, `line_node`에 통합), 색 마커 인식·분기 회전, 유턴, 앱 영상 끄기 | 사용자 실차 확인: "마커 테스트와 u턴까지는 완료" | 확정 — 태그 생성 대기 |
| BL-4 | — | (미커밋) | `bl4-mission` 예정 | 새 사양 — 초기 설정(트랙 매핑), 모양별 상차·하차, 7세그먼트, 표지판 분류, `mission_node` | PC 시험만 (아래 5절) | **후보** — 실차 확인 후 확정 |

- BL-0~3은 이미 GitHub에 있는 커밋이다. 태그만 붙이면 된다 (패키지 버전은 모두 `0.1.0`이었음)

---

## 4. 베이스라인 기록 방법

### 확정할 때 (실차 확인 후)
1. 작업 트리가 깨끗한지 확인 — `git status` (07_final/panel 같은 보류 항목 제외)
2. PC에서 전체 pytest, 앱 + 가짜 차량 시험
3. Pi에서 `git pull` → `colcon build` → `colcon test` → 해당 S단계 실차 시험
4. 아래 "Pi 환경 기록"과 시험 결과를 이 문서 3절·`04_unit/<기능>/test.md`에 적음
5. 패키지 버전 확인(`0.<번호>.0`) → 주석 태그 → push

```bash
git tag -a bl4-mission -m "BL-4: 초기 설정·작업 — S10 실차 확인 (날짜, 근거)"
```
```bash
git push origin bl4-mission
```

### Pi 환경 기록
```bash
git -C ~/logistics-automation describe --tags --always
```
```bash
lsb_release -ds; printenv ROS_DISTRO; python3 --version
```
```bash
python3 -c "import cv2, numpy; print(cv2.__version__, numpy.__version__)"
```
```bash
python3 -c "import importlib.metadata as m; print(m.version('ai-edge-litert'))"
```
```bash
git -C ~/camera_ws/src/camera_ros log -1 --format=%h
```
```bash
sha256sum ~/logistics-automation/ros2_ws/src/agv_bringup/models/*
```

### 되돌리기 (문제가 생겼을 때)
```bash
git checkout bl3-marker-uturn
```
```bash
rm -rf build install log
```
```bash
colcon build
```
- 인터페이스(msg·srv·action)가 다른 베이스라인으로 옮길 때는 빌드 폴더를 지우고 다시 빌드한다
- 노트북 앱도 같은 베이스라인으로 맞춘다 (블루투스 메시지가 다를 수 있음) — BL-3 이하는 `SETUP`·`START <원> <사각>`·`MAP`을 모름
- 비교가 끝나면 `git checkout main`

---

## 5. BL-4 후보 내용 (2026-11-02)

| 패키지 | 버전 | 주요 변경 |
|---|---|---|
| `agv_interfaces` | 0.4.0 | `Status.msg`, `Command.msg`, `srv/Classify.srv` 추가, `Drive.action`에 `skip_stop_lines` |
| `agv_protocol` | 0.4.0 | `SETUP`, `START <원> <사각>`, `STATUS <state> <mode> <done>/<total> <cargo> <load> <target>`, `MAP` |
| `agv_hw` | 0.4.0 | `segment_node`, 7세그먼트 드라이버(GPIO 4·27·12·13·9·10·11) |
| `agv_vision` | 0.4.0 | `classifier_node`(Teachable Machine 표지판, LiteRT) |
| `agv_control` | 0.4.0 | `mission_node`(초기 설정·작업 상태 머신), drive `APPROACH`·유턴 뒤 정지선 통과 |
| `agv_bt` | 0.4.0 | mission 연동(`standalone=false`), `MAP` 전송, 매핑 없으면 `START` 거부 |
| `agv_bringup` | 0.4.0 | `agv.launch.py`, `models/`, params(classifier·mission·segment) |
| `laptop_app` | — | 사용자 탭(초기 설정·매핑 표·모양별 개수), 가짜 차량이 실제 상태 머신 사용 |

- **PC 시험:** pytest 403개 통과 (protocol 111, hw 35, bt 20, vision 122, control 115), 앱 + 가짜 차량으로 초기 설정 → 작업(원 2·사각 1) → 복귀
- **확정 조건:** Pi 빌드·`colcon test`, 표지판 모델 넣기(SHA-256 기록), 7세그먼트 배선, S6(표지판 앞 정지·인식), S10(초기 설정 3회·작업 3회) 실차 통과
- **알려진 문제:** 주행 중 Pi 꺼짐(전원 의심, 원인 미확정) — 확정 전에 해결 또는 기록 필요

---

## 6. 변경 관리 규칙

| 변경 종류 | 함께 바꿀 것 | 버전 |
|---|---|---|
| 인터페이스 (msg·srv·action, 블루투스 메시지) | 차량 코드 + 앱 + 규약 문서 + 시험을 **한 커밋**에 | 다음 베이스라인(`0.<번호+1>.0`) |
| 기능 추가 (새 노드·새 동작) | 기능 설계서·구조 문서·launch·params | 다음 베이스라인 |
| 버그 수정·조정값 변경 | 시험, HISTORY | 같은 베이스라인 수정 번호(`0.<번호>.<수정+1>`) |
| AI 모델 교체 | 모델 파일 SHA-256, 인식률 기록 | 같은 베이스라인 수정 번호 |
| 문서만 | — | 버전 바꾸지 않음 |

- 실차에서 확인되지 않은 변경 위에 태그를 붙이지 않는다
- 실차 시험 결과가 나쁘면 직전 베이스라인 태그로 되돌려 같은 시험을 해 보고, 어느 변경 때문인지 비교한다

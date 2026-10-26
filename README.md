# ROS 2와 Teachable Machine을 활용한 비전 인식 물류 AGV

작업장에서 카메라와 AI로 화물 종류(원/사각형 이미지)를 인식하고, 종류별로 정해진 하차 지점까지 라인트레이싱으로 자동 운반·분류하는 소형 물류 자동화 차량(AGV)입니다.

## 구성
| 구분 | 내용 |
|------|------|
| 하드웨어 | JKEMC PiCar-R5 키트 (Raspberry Pi 5, DRV8835 × 2 모터 드라이버, PCA9685 서보, TT 모터 4개, CSI 카메라 + 팬틸트) + 초음파 센서, LED |
| 차량 소프트웨어 | Ubuntu 24.04 LTS, ROS 2 Jazzy, Python (rclpy) |
| AI | Teachable Machine → TFLite |
| 통신 | Raspberry Pi 5 내장 블루투스 ↔ 노트북 Python 앱 (작업 입력·비상 정지·수동 제어·영상) |

## 문서
- [개요서](01_planning/overview.md) — 목적, 시스템 구성, 기능, 동작 시나리오, 예외 처리, 리스크
- [진행 프로세스](PROCESS.md) — 7단계(기획 → 설계 → 준비 → 단위 개발 → 통합 → 테스트 → 마무리)
- [GPIO 핀맵](02_design/hardware/pinmap.md) · [브레드보드 배선도](02_design/hardware/wiring.md)
- [기능 설계서](02_design/software/functional_design.md) — ROS 2 노드·토픽, 상태 머신, 기능별 상세
- [소프트웨어 구조 설계](02_design/software/software_architecture.md) — 계층, 패키지·모듈, 실행 구성, 테스트·개발 흐름
- [클래스 설계](02_design/software/class_design.md) — 공통 타입, 클래스·메서드, 상태 머신 이벤트/동작, 시퀀스
- [블루투스 통신 규약](02_design/communication/bluetooth_protocol.md) · [Teachable Machine 클래스](02_design/ai/tm_classes.md)
- [단계별 테스트 트랙](02_design/track/test_track.md)
- [개발 순서 및 절차](02_design/development_plan.md) — 작업 흐름·주차별 계획·관문·기능 개발 절차·Git 규칙 · [주차별 실행 체크리스트](02_design/weekly_checklist.md)

## 폴더 구조
```
├─ 01_planning/        개요서
├─ 02_design/          하드웨어·소프트웨어·통신·AI·트랙 설계
├─ 03_setup/           환경 구축·부품 준비 기록
├─ 04_unit/            단위 기능 테스트 기록
├─ 05_integration/     통합 기록
├─ 06_scenario_test/   시나리오 테스트 결과
├─ 07_final/           보고서·발표 자료
├─ ros2_ws/src/        차량 코드 (ROS 2 Python 패키지)
└─ laptop_app/         노트북 앱 (Python)
```

## 현재 상태
| 1 기획 | 2 설계 | 3 준비 | 4 단위 개발 | 5 통합 | 6 테스트 | 7 마무리 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| 완료 | **진행 중** | 대기 | 대기 | 대기 | 대기 | 대기 |

- 완료: 개요서, 하드웨어 선정(PiCar-R5 키트), 모터·서보 핀맵, 정지 지점 판단 방식(검은 가로 정지선 + 표지판 확인), 경로 표시 방식(검은 라인 + 분기점 목적지 색 마커)
- 완료: [단계별 테스트 트랙](02_design/track/test_track.md) 설계 확정 (S0~S10, I자 통합 트랙) — 검증 결과를 반영해 실제 트랙 설계 예정
- 완료: [브레드보드 배선도](02_design/hardware/wiring.md) (초음파·LED 핀 배정)
- 완료: [기능 설계서](02_design/software/functional_design.md) (ROS 2 노드·토픽·상태 머신), [블루투스 통신 규약](02_design/communication/bluetooth_protocol.md), [Teachable Machine 클래스](02_design/ai/tm_classes.md)
- 완료: [부품 목록](02_design/bom.md), [소프트웨어 구조 설계](02_design/software/software_architecture.md), [클래스 설계](02_design/software/class_design.md)
- 완료: [개발 순서 및 절차](02_design/development_plan.md) (주차별 계획 W1~W4, 관문 G1~G4)
- 남은 일: 보유 부품·Pi 5 메모리 확인 후 2단계 완료 → 3단계(준비)

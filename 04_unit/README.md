# 4단계 · 단위 기능 개발
기능별 폴더에는 **테스트 기록(test.md)** 과 단독 테스트용 스크립트를 둔다.
실제 차량 코드는 `ros2_ws/src/`(ROS 2 패키지), 노트북 코드는 `laptop_app/`에 작성한다.
번호는 권장 개발 순서.
- 01_camera — 키트 카메라 + Ubuntu + camera_ros 동작 확인, 팬틸트 서보(PCA9685)로 각도 전환 (가장 큰 리스크라 먼저)
- 02_motor — 제어보드 DRV8835 × 2, 바퀴 4개 개별 제어(lgpio 소프트웨어 PWM), 전진/후진/회전/유턴
- 03_line_tracing — 카메라 기반 라인 추종
- 04_color_marker — 분기점 목적지 색 마커 판별 (빨강·초록·파랑·노랑)
- 05_ultrasonic — 거리 측정, 적재 거리(3초 유지)·장애물 정지
- 06_led — 빨강 = 원, 초록 = 사각형
- 07_bluetooth_control — 작업 횟수 입력, 비상 정지, 수동 제어
- 08_ai_vision — Teachable Machine 학습(dataset/, model/), 화물·표지판 인식
- 09_bluetooth_video — 블루투스 영상 전송, 속도·지연 측정
- 10_lcd — (옵션) 작업 횟수 표시

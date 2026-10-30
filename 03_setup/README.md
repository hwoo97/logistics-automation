# 3단계 · 준비
- 부품 구매·수령 체크리스트 (PiCar-R5 키트, Pi 5, 초음파, LED, 저항, 브레드보드)
- Pi 5에 Ubuntu 24.04 LTS 설치, ROS 2 Jazzy 설치 기록
- Camera Module 3 Wide 동작 확인 (Ubuntu용 libcamera + camera_ros) — 최우선 → [camera/](camera/README.md) (설치 절차·점검 스크립트·결과)
- lgpio 설치(모터), I2C 활성화(PCA9685 서보) → [gpio_i2c/](gpio_i2c/README.md) (LED 점등·PCA9685·서보 채널 점검), 블루투스 페어링 → [bluetooth/](bluetooth/README.md) (RFCOMM 에코 시험 스크립트·절차·결과)
- TFLite 추론 런타임 확인 (Python 3.12 호환) → [ai/](ai/README.md) (Teachable Machine 모델 → LiteRT 추론 시간·인식률 점검)
- 노트북 Python 환경, Teachable Machine 준비
- 트랙 제작 기록

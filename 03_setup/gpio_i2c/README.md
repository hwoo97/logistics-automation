# lgpio LED 점등 · I2C PCA9685 확인

> W1 준비 항목 ([주차별 실행 체크리스트](../../02_design/weekly_checklist.md)) — 담당 A
> 목표: 모터·LED·초음파 노드의 바탕인 **lgpio**와 서보(팬틸트) 노드의 바탕인 **I2C + PCA9685**가 Ubuntu에서 동작하는지 확인 → 관문 G1
> 핀: [GPIO 핀맵](../../02_design/hardware/pinmap.md) · [브레드보드 배선도](../../02_design/hardware/wiring.md)

| 파일 | 역할 |
|------|------|
| [led_check.py](led_check.py) | 헤더 GPIO 칩(RP1)을 자동으로 찾아 빨강 GPIO16 → 초록 GPIO26 차례로 깜박임 |
| [pca9685_check.py](pca9685_check.py) | I2C 버스 스캔 → PCA9685(0x40) 레지스터 읽기 → (`--channel`) 서보를 50Hz로 각도 이동 |

## 1. 설치·권한
```bash
sudo apt update
sudo apt install -y python3-lgpio python3-smbus2 i2c-tools
ls -l /dev/gpiochip* /dev/i2c-*      # 각 장치의 그룹 확인 (예: dialout, gpio, i2c)
id -nG                               # 내 그룹
```
- 장치 그룹에 내가 없으면: `sudo usermod -aG <그룹> $USER` → 로그아웃·재로그인 (예: `i2c`, `dialout`)
- 그래도 권한 오류면 `sudo python3 ...`로 한 번 시험해 원인이 권한인지 구분

## 2. I2C 활성화
- `/dev/i2c-1`이 없으면 `/boot/firmware/config.txt`에 아래 줄이 있는지 확인 → 없으면 추가 후 재부팅
  ```
  dtparam=i2c_arm=on
  ```
- 제어보드 **배터리 스위치를 켠다** — PCA9685는 제어보드 전원이 있어야 응답한다
```bash
i2cdetect -y 1          # 표에 40 이 보이면 PCA9685 응답
```

## 3. LED 점등 (lgpio)
브레드보드 배선: GPIO16(핀 36) → 330Ω → 빨강 LED(+), GPIO26(핀 37) → 220~330Ω → 초록 LED(+), LED(−) → GND
```bash
cd ~/logistics-automation/03_setup/gpio_i2c
python3 led_check.py
```
- 처음에 찾은 gpiochip 목록과 사용할 칩(`pinctrl-rp1` 라벨)이 출력된다. Pi 5는 커널에 따라 `gpiochip0` 또는 `gpiochip4` → **번호를 결과표에 기록** (드라이버 설정에 사용)
- 빨강 3회 → 초록 3회 → 둘 다 2초 켜짐 순서인지 눈으로 확인

## 4. PCA9685 확인 (I2C)
```bash
python3 pca9685_check.py                 # 스캔 + 레지스터 읽기 (서보 안 움직임)
```
- `응답 주소: 0x40 ...`, `MODE1=0x..` 가 나오면 통과. 0x70(전체 호출 주소)이 함께 보일 수 있다

## 5. 서보 채널 확인 (팬틸트)
팬틸트 서보를 제어보드 서보 헤더에 연결한 상태에서, **채널을 하나씩** 시험해 어느 헤더가 팬/틸트인지 찾는다.
```bash
python3 pca9685_check.py --channel 0     # 90° → 60° → 120° → 90° 후 힘 풀림
python3 pca9685_check.py --channel 1
```
- 팬틸트가 끝까지 닿아 걸리지 않는지 보면서 시험. 걸리면 `--angles 90 80 100 90`처럼 범위를 좁힌다
- **I2C는 응답하는데 서보가 안 움직이면**: `--oe-low` 추가 → GPIO17을 LOW로 두고 다시 시험
  - 이걸로 움직이면 **GPIO17 = PCA9685 OE**로 확정 (핀맵의 GPIO17 예약 해소) → 결과표에 기록
- 헤더 번호 ↔ 채널 번호, 팬/틸트 위치를 [핀맵](../../02_design/hardware/pinmap.md) 서보 표에 반영

## 6. 통과 기준 (G1)
| # | 항목 | 통과 기준 |
|:---:|------|------|
| 1 | lgpio | `led_check.py` 오류 없음, 헤더 칩 번호 확인 |
| 2 | LED | 빨강·초록이 이름대로 깜박임 |
| 3 | I2C | `i2cdetect -y 1`과 `pca9685_check.py`에서 0x40 응답 |
| 4 | 서보 | 팬·틸트 서보가 지정 각도로 움직임, 채널 번호 확인 |
| 5 | GPIO17 | OE 여부 확인 (필요 없음 / OE 확정) |

## 7. 문제 해결
| 증상 | 확인·조치 |
|------|-----------|
| `lgpio 없음` | `sudo apt install python3-lgpio` |
| `gpiochip 열기 실패` | 1절 권한·그룹, `sudo`로 시험 |
| `RP1 헤더 GPIO 칩을 찾지 못함` | `sudo apt install gpiod` → `gpiodetect`로 칩 확인 → `--chip N` |
| LED가 안 켜짐 | LED 방향(긴 다리 +), 저항, 헤더 핀 번호(BCM 16 = 물리 36), GND 연결 |
| `/dev/i2c-1 열기 실패` | 2절 `dtparam=i2c_arm=on`, 재부팅, i2c 그룹 |
| 0x40 응답 없음 | 제어보드 배터리 스위치, 보드와 Pi 결합 상태, `i2cdetect -y 1`에서 다른 주소가 보이면 `--addr 0x..` |
| 서보가 떨리거나 약함 | 배터리 전압 확인(7.4V 근처), 서보 전원(V)이 제어보드에서 공급되는지 |

## 8. 결과 기록
| 날짜 | 담당 | 항목 | 결과 | 비고 |
|------|------|:---:|------|------|
| | | 1 | 헤더 칩: gpiochip___ | |
| | | 2 | | |
| | | 3 | 응답 주소: | |
| | | 4 | 팬 = 채널 ___ (헤더 ___), 틸트 = 채널 ___ (헤더 ___) | |
| | | 5 | GPIO17: | |

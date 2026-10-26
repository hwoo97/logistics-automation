# 블루투스 페어링 · RFCOMM 에코 시험

> W1 준비 항목 ([주차별 실행 체크리스트](../../02_design/weekly_checklist.md)) — 담당 C
> 목적: Pi 5 ↔ 노트북이 [블루투스 규약](../../02_design/communication/bluetooth_protocol.md)대로 RFCOMM 채널 1·2로 통신할 수 있는지, 지연·처리량이 충분한지 **개발 전에** 확인한다.

| 파일 | 실행 위치 | 역할 |
|------|-----------|------|
| [rfcomm_echo_server.py](rfcomm_echo_server.py) | Pi 5 (Ubuntu 24.04) | 받은 줄을 되돌려 보냄, `BULK n` 요청에 n바이트 전송 |
| [rfcomm_echo_client.py](rfcomm_echo_client.py) | 노트북 (Windows / Ubuntu) | 연결 시간·에코 왕복 시간(RTT)·Pi → 노트북 처리량 측정 후 판정 |

- 표준 라이브러리만 사용 (Python 3.12 — Ubuntu 24.04 기본, Windows는 python.org 3.12 이상)
- 둘 다 `--tcp` 옵션으로 블루투스 없이 로직만 확인할 수 있다 (2026-10-26 TCP로 동작 확인 완료)

## 준비물
- Pi 5 (Ubuntu 24.04 설치 완료), 노트북 앱을 실행할 **팀 노트북** (블루투스 내장, 없으면 USB 동글) — 시연 때 쓸 노트북으로 시험한다
- 두 기기에 이 폴더의 스크립트 (`git pull`), 노트북에 Python 3.12 이상

## 1. Pi 5 확인
```bash
bluetoothctl show                 # Controller 주소(XX:XX:...)가 나오면 정상 → 주소 기록
systemctl status bluetooth        # active (running)
python3 -c "import socket; print(socket.AF_BLUETOOTH, socket.BTPROTO_RFCOMM)"
```
- Controller가 없으면: `sudo apt install pi-bluetooth` (패키지가 있으면) → 재부팅 후 다시 확인

## 2. 페어링 (한 번만)
**Pi** — `bluetoothctl` 실행 후 입력:
```
power on
agent on
default-agent
pairable on
discoverable on
```
**노트북 (Windows)** — 설정 → Bluetooth 및 장치 → 장치 추가 → Bluetooth → Pi 이름 선택
- 양쪽에 같은 숫자가 뜨면 Windows에서 "연결", Pi `bluetoothctl`에서 `Confirm passkey ... (yes/no)` → `yes`
- `Authorize service` 질문이 나오면 `yes`

**Pi** — 재연결 때 묻지 않도록 신뢰 등록 후 검색 끄기:
```
devices                     # 노트북 주소 확인
trust <노트북 주소>
discoverable off
quit
```
- 노트북이 Ubuntu면 `bluetoothctl`에서 `scan on` → `pair <Pi 주소>` → `trust <Pi 주소>`

## 3. 에코 시험
**Pi**
```bash
cd ~/logistics-automation/03_setup/bluetooth
python3 rfcomm_echo_server.py              # 채널 1 (제어용)
```
**노트북**
```bash
python rfcomm_echo_client.py <Pi 주소>
```
출력 예:
```
연결 성공 (1.20초)
에코 100/100 성공, 유실 0
RTT(ms) 최소 25.0 / 평균 40.2 / 95% 60.1 / 최대 80.3
처리량 Pi→노트북: 1000000바이트 8.10초 = 988 kbps
[판정] ...
```
- 채널 2(영상용)도 확인: Pi `--channel 2`, 노트북 `--channel 2`
- 두 채널을 동시에 쓰는지 확인: Pi에서 서버 2개(채널 1, 2)를 각각 띄우고 노트북에서 클라이언트 2개를 동시에 실행

## 4. 시험 항목과 통과 기준
| # | 항목 | 방법 | 통과 기준 |
|:---:|------|------|------|
| 1 | 페어링 | 2절 | Windows 장치 목록에 Pi 표시, Pi `devices`에 노트북 표시 |
| 2 | 채널 1 에코 | 기본 실행 (100회) | 유실 0, 최대 RTT 500ms 미만 (하트비트 1.5초의 1/3) |
| 3 | 채널 2 에코 | `--channel 2` | 2번과 같음 |
| 4 | 두 채널 동시 | 3절 마지막 | 두 클라이언트 모두 2번 기준 통과 |
| 5 | 처리량 | 거리 1m·3m에서 각각 | 영상 필요량(약 600kbps)의 1.5배 이상 = 충분, 1~1.5배 = 빠듯 → 영상 해상도·FPS 조정 검토 |
| 6 | 재연결 | 클라이언트 종료 후 다시 실행 | 다시 페어링 없이 연결 |
| 7 | 재부팅 후 | Pi 재부팅 → 서버 실행 → 연결 | 다시 페어링 없이 연결 |

- 처리량이 부족하면 W2 영상 채널 설계 전에 [개발 순서 및 절차](../../02_design/development_plan.md) G2 대체안(해상도·FPS 축소 → Wi-Fi 검토)을 미리 논의한다.

## 5. 문제 해결
| 증상 | 확인·조치 |
|------|-----------|
| 서버 `Address already in use` | 그 채널을 다른 서비스가 사용 중 → `--channel 3` 등 다른 채널로 시험, 결과에 기록 |
| 서버 `Permission denied` | `sudo python3 ...`로 한 번 시험 → 되면 사용자를 `bluetooth` 그룹에 추가 |
| Windows `[WinError 10060]`·`10064` (연결 시간 초과·거부) | Pi 서버 실행 중인지, 채널 번호 일치, 페어링 상태, 거리 확인 |
| Windows에서 채널 지정 연결이 계속 실패 | Pi에 SPP 서비스 등록 필요할 수 있음 → `/lib/systemd/system/bluetooth.service`의 `ExecStart`에 `--compat` 추가 → `sudo systemctl daemon-reload && sudo systemctl restart bluetooth` → `sudo sdptool add --channel=1 SP` |
| 노트북에 블루투스 없음 | USB 블루투스 동글(4.0 이상) 사용 |

## 6. 결과 기록
| 날짜 | 담당 | 노트북 (OS·블루투스) | 항목 | 결과 | 비고 |
|------|------|------|:---:|------|------|
| 2026-10-26 | Claude | 문서 작성용 PC (Windows 11, Python 3.12) | 로직 | TCP 모드 에코 50/50, BULK 2MB 정상 | 스크립트 로직 확인만. 실제 시험은 팀 노트북 ↔ Pi 5 |
| | | | 1 | | |
| | | | 2 | | |
| | | | 3 | | |
| | | | 4 | | |
| | | | 5 | 1m: ___ kbps / 3m: ___ kbps | |
| | | | 6 | | |
| | | | 7 | | |

# Camera Module 3 Wide 동작 확인 (Ubuntu 24.04 + ROS 2 Jazzy)

> W1 준비 항목 1순위 ([주차별 실행 체크리스트](../../02_design/weekly_checklist.md)) — 담당 A
> 목표: Pi 5에서 `camera_ros`가 `/camera/image_raw`를 발행하고, 라인 추종에 충분한 속도(15fps 이상)가 나오는지 확인 → 관문 G1

## 왜 따로 설치가 필요한가
- Ubuntu 24.04·ROS 2 기본 libcamera(업스트림)는 **Pi 5(PiSP)와 Camera Module 3 지원이 불완전**하다.
- Raspberry Pi가 관리하는 **libcamera 포크**(`raspberrypi/libcamera`)를 써야 한다. 그 위에 `camera_ros`를 올린다.
- Ubuntu 기본 libcamera(`libcamera0.2`, `ros-jazzy-libcamera`)가 함께 깔리면 충돌하므로 주의한다.

| 파일 | 역할 |
|------|------|
| [camera_precheck.sh](camera_precheck.sh) | 설치 전·후 상태 점검 (OS, video 그룹, config.txt, 센서 감지, 장치, libcamera, 카메라 목록, ROS) — 아무것도 바꾸지 않음 |
| [camera_topic_check.py](camera_topic_check.py) | `camera_ros` 실행 중 토픽 해상도·인코딩·FPS·끊김 측정, 마지막 프레임 JPEG 저장 |

## 0. 연결
1. Pi 전원을 끈 상태에서 연결한다.
2. 케이블 **22핀(좁은) 쪽 → Pi 5 CAM/DISP 포트**(0 또는 1), **15핀 쪽 → 카메라**. 금속 접점 방향을 맞추고 잠금 탭을 끝까지 누른다.
3. 어느 포트(0/1)에 꽂았는지 기록한다.

## 1. 사전 점검
```bash
cd ~/logistics-automation/03_setup/camera
bash camera_precheck.sh
```
- `[확인]` 항목을 보고 아래 단계를 진행한다. 설치 후에도 다시 실행해 비교한다.
- `video` 그룹이 없으면: `sudo usermod -aG video $USER` → 로그아웃·재로그인

## 2. 센서 감지 (config.txt)
- `/boot/firmware/config.txt`에 `camera_auto_detect=1`이 있으면 그대로 둔다 → 재부팅 후 `dmesg | grep imx708`
- 감지되지 않으면 수동 지정:
  ```
  camera_auto_detect=0
  [all]
  dtoverlay=imx708          # 포트 1이면 dtoverlay=imx708,cam1
  ```
  저장 → `sudo reboot` → `dmesg | grep imx708`

## 3. libcamera 설치 — 방법 A (권장: 공식 포크 소스 빌드, 약 10~20분)
```bash
sudo apt update
sudo apt install -y git meson ninja-build cmake pkg-config python3-jinja2 python3-yaml python3-ply \
  libboost-dev libgnutls28-dev openssl libtiff-dev libyaml-dev libevent-dev libdrm-dev libjpeg-dev \
  libglib2.0-dev nlohmann-json3-dev
cd ~
git clone https://github.com/raspberrypi/libcamera.git
cd libcamera
meson setup build --buildtype=release \
  -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp \
  -Dcam=enabled -Dqcam=disabled -Dgstreamer=disabled -Dpycamera=disabled \
  -Dtest=false -Dlc-compliance=disabled -Ddocumentation=disabled
ninja -C build
sudo ninja -C build install
sudo ldconfig
```
- 설치 위치는 `/usr/local`. `pkg-config --modversion libcamera`로 버전 확인
- meson 단계에서 패키지가 없다고 나오면 그 이름을 `apt install`로 추가 후 다시 실행

### 방법 B (빠름, 비공식 PPA)
```bash
sudo add-apt-repository ppa:marco-sonic/rasppios
sudo apt update && sudo apt upgrade
sudo apt install libcamera-tools rpicam-apps-lite
```
- Raspberry Pi 공식이 아닌 개인 PPA라 내용이 검증되지 않았다. **팀이 동의할 때만** 사용하고 결과표에 기록한다.
- 이 방법은 `rpicam-hello`·`rpicam-jpeg` 명령도 함께 설치된다.

## 4. 카메라 단독 확인 (ROS 없이)
```bash
cam -l                      # 방법 A: 'imx708_wide' 가 보이면 성공
cam -c 1 -C 30              # 프레임 30장 받기 (오류 없이 끝나면 성공)
rpicam-hello --list-cameras # 방법 B
rpicam-jpeg -n -o test.jpg  # 방법 B: 사진 1장 (모니터 없이 -n)
```
- 목록에 `imx708_wide`(광각)가 나와야 한다. `imx708`만 나오면 일반형으로 인식된 것 → 결과표에 기록

## 5. camera_ros 설치 (별도 워크스페이스)
프로젝트 `ros2_ws`와 분리해 `~/camera_ws`에 둔다 (외부 패키지를 레포에 넣지 않기 위해).
```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/camera_ws/src && cd ~/camera_ws/src
git clone https://github.com/christianrauch/camera_ros.git
cd ~/camera_ws
rosdep install -y --from-paths src --ignore-src --rosdistro jazzy --skip-keys=libcamera
colcon build --event-handlers=console_direct+
source ~/camera_ws/install/setup.bash
```
- `--skip-keys=libcamera`: 3단계에서 설치한 포크 libcamera를 Ubuntu 기본 것으로 덮어쓰지 않게 한다
- 빌드 후 `bash camera_precheck.sh`에서 `ros-jazzy-libcamera`·`libcamera0.2` 경고가 없어야 한다
- 매번 쓰려면 `~/.bashrc`에 `source ~/camera_ws/install/setup.bash` 추가 (프로젝트 `ros2_ws`보다 먼저)

## 6. 영상 토픽 확인
**터미널 1** — 카메라 노드 (640×480, 30fps 요청)
```bash
ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480 -p FrameDurationLimits:="[33333,33333]"
```
**터미널 2**
```bash
ros2 topic list | grep camera            # /camera/image_raw 확인
ros2 topic hz /camera/image_raw
cd ~/logistics-automation/03_setup/camera
python3 camera_topic_check.py --seconds 10
```
- 저장된 `captures/camera_check.jpg`를 노트북으로 가져와 확인 (`captures/`는 GitHub에 올라가지 않음)
- 라인 높이(차량 위, 아래를 보는 각도)에서 검은 테이프를 비춰 한 장 더 저장: `--save captures/line.jpg`

## 7. 초점 (자동 초점 → 고정)
주행 중에는 초점이 흔들리지 않게 고정한다 ([기능 설계서](../../02_design/software/functional_design.md) F1).
```bash
ros2 param list /camera | grep -iE 'Af|Lens'          # AfMode, LensPosition 등이 보이는지
ros2 param set /camera AfMode 0                       # 0 = 수동
ros2 param set /camera LensPosition 3.0               # 단위: 디옵터(1/m), 3.0 ≈ 33cm 거리
```
- LensPosition을 바꿔 가며 라인 거리(약 20~40cm)와 표지판 거리(25~45cm)에서 모두 선명한 값을 찾는다 → 결과표에 기록
- 파라미터가 보이지 않으면 카메라 노드 시작 시 `-p AfMode:=0 -p LensPosition:=3.0`으로 시험

## 8. 통과 기준 (G1)
| # | 항목 | 통과 기준 |
|:---:|------|------|
| 1 | 센서 감지 | `dmesg`에 imx708, `cam -l`(또는 `rpicam-hello --list-cameras`)에 `imx708_wide` |
| 2 | 단독 캡처 | `cam -c 1 -C 30` 또는 `rpicam-jpeg` 오류 없음 |
| 3 | ROS 토픽 | `/camera/image_raw` 발행, 640×480 |
| 4 | 속도 | `camera_topic_check.py` FPS 15 이상, 최대 간격 200ms 미만 |
| 5 | 화질 | 저장 사진에서 검은 테이프 경계가 선명, 색 정상, 방향 정상(뒤집혔으면 `orientation` 파라미터) |
| 6 | 초점 고정 | AfMode 수동 + LensPosition 값 확정 |

- 2일 안에 1~4번이 안 되면 **USB 웹캠으로 전환**을 팀이 결정한다 ([개발 순서 및 절차](../../02_design/development_plan.md) 5절)

## 9. 문제 해결
| 증상 | 확인·조치 |
|------|-----------|
| `dmesg`에 imx708 없음 | 케이블 방향·잠금, 다른 포트, 2절 수동 오버레이, 카메라 쪽 커넥터도 확인 |
| `cam -l` 목록 비어 있음 | `video` 그룹, `/dev/dma_heap` 권한, 포크 libcamera가 쓰이는지(`ldd $(which cam) \| grep libcamera` → `/usr/local`) |
| camera_ros 빌드 시 libcamera 버전 충돌 | `sudo apt remove ros-jazzy-libcamera libcamera0.2` 후 다시 빌드, `--skip-keys=libcamera` 확인 |
| 노드 실행 시 `no cameras available` | 4단계부터 다시 확인 (ROS 이전 문제) |
| FPS가 낮음 | 해상도 낮추기(`320×240`), `FrameDurationLimits` 확인, 다른 무거운 프로세스 확인(`htop`) |
| 사진이 흐림 | 7절 LensPosition 조정, 렌즈 보호 필름 제거 |

## 10. 결과 기록
| 날짜 | 담당 | 설치 방법(A/B) | 포트 | 항목 | 결과 | 비고 |
|------|------|:---:|:---:|:---:|------|------|
| | | | | 1 | | |
| | | | | 2 | | |
| | | | | 3 | | |
| | | | | 4 | FPS ___, 최대 간격 ___ms | |
| | | | | 5 | | |
| | | | | 6 | LensPosition ___ | |

참고: [camera_ros](https://github.com/christianrauch/camera_ros) · [Raspberry Pi libcamera 포크](https://github.com/raspberrypi/libcamera) · [ROS Discourse — Pi 5 + Jazzy 카메라 설치](https://discourse.openrobotics.org/t/installation-and-configuration-of-the-raspberry-pi-camera-on-a-ros-2-jazzy-raspberry-pi-5/45177)

#!/usr/bin/env bash
# 카메라 사전 점검 — Raspberry Pi 5 (Ubuntu 24.04)에서 실행
#   bash camera_precheck.sh
# 설치 전·후 어디서 막혔는지 한 번에 보여준다. 아무것도 바꾸지 않는다.

ok()   { echo "  [OK]   $*"; }
warn() { echo "  [확인] $*"; }

echo "== OS"
echo "  $(lsb_release -ds 2>/dev/null) / 커널 $(uname -r)"
grep -q "Raspberry Pi 5" /proc/device-tree/model 2>/dev/null && ok "$(tr -d '\0' < /proc/device-tree/model)" || warn "Raspberry Pi 5가 아님"

echo "== 사용자 그룹"
id -nG | grep -qw video && ok "video 그룹 포함" || warn "video 그룹 없음 → sudo usermod -aG video \$USER 후 재로그인"

echo "== /boot/firmware/config.txt"
CFG=/boot/firmware/config.txt
if [ -f "$CFG" ]; then
  grep -nE '^\s*(camera_auto_detect|dtoverlay=imx)' "$CFG" | sed 's/^/  /' || true
  grep -qE '^\s*camera_auto_detect=1' "$CFG" || grep -qE '^\s*dtoverlay=imx708' "$CFG" \
    && ok "카메라 자동 감지 또는 imx708 오버레이 설정됨" \
    || warn "camera_auto_detect=1 또는 dtoverlay=imx708 없음"
else
  warn "$CFG 없음"
fi

echo "== 커널 로그 (imx708)"
LOG=$(dmesg 2>/dev/null || sudo dmesg)
if echo "$LOG" | grep -qi imx708; then
  echo "$LOG" | grep -i imx708 | tail -n 3 | sed 's/^/  /'
  ok "센서 감지"
else
  warn "imx708 메시지 없음 → 케이블 방향·잠금, 포트(CAM/DISP 0·1), config.txt 확인"
fi

echo "== 장치 파일"
ls /dev/media* >/dev/null 2>&1 && ok "$(ls /dev/media* | tr '\n' ' ')" || warn "/dev/media* 없음"
ls /dev/dma_heap/* >/dev/null 2>&1 && ok "dma_heap: $(ls /dev/dma_heap | tr '\n' ' ')" || warn "/dev/dma_heap 없음"

echo "== libcamera"
pkg-config --modversion libcamera >/dev/null 2>&1 \
  && ok "libcamera $(pkg-config --modversion libcamera) ($(pkg-config --variable=prefix libcamera))" \
  || warn "pkg-config에서 libcamera를 찾지 못함 (설치 전이거나 PKG_CONFIG_PATH 문제)"
dpkg -l 2>/dev/null | awk '/libcamera/ {print "  " $1, $2, $3}'
dpkg -l 2>/dev/null | grep -qE 'ros-jazzy-libcamera|libcamera0\.2' \
  && warn "Ubuntu/ROS 기본 libcamera가 함께 설치됨 → 충돌 가능 (README 6절)"

echo "== 카메라 목록"
if command -v rpicam-hello >/dev/null; then
  rpicam-hello --list-cameras 2>&1 | sed 's/^/  /'
elif command -v cam >/dev/null; then
  cam -l 2>&1 | sed 's/^/  /'
else
  warn "rpicam-hello·cam 명령 없음 (libcamera 설치 전)"
fi

echo "== ROS 2"
if [ -n "$ROS_DISTRO" ]; then
  ok "ROS_DISTRO=$ROS_DISTRO"
  ros2 pkg prefix camera_ros >/dev/null 2>&1 && ok "camera_ros: $(ros2 pkg prefix camera_ros)" || warn "camera_ros 없음"
else
  warn "ROS 환경 미설정 → source /opt/ros/jazzy/setup.bash"
fi

# ROS 2 워크스페이스 (Pi 5 · Ubuntu 24.04 · ROS 2 Jazzy · Python)
- 차량에서 실행되는 모든 코드는 `src/` 아래 ROS 2 Python 패키지(ament_python)로 작성한다.
- 패키지·노드·토픽 구성은 2단계 설계(`02_design/software/`)에서 확정한다.
- 빌드: `colcon build --symlink-install` → `source install/setup.bash`

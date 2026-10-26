# ROS 2 워크스페이스 (Pi 5 · Ubuntu 24.04 · ROS 2 Jazzy · Python)
- 차량 코드는 `src/` 아래 ROS 2 패키지로 작성한다: agv_interfaces · agv_protocol · agv_hw · agv_vision · agv_control · agv_bt · agv_bringup
- 패키지·모듈·클래스 구조: `02_design/software/software_architecture.md`
- 노드·토픽·상태 머신: `02_design/software/functional_design.md`
- 빌드: `colcon build --symlink-install` → `source install/setup.bash`
- 실행: `ros2 launch agv_bringup <단계>.launch.py` (s0_hw ~ agv)

"""전체 운용 — S0 하드웨어(+ 7세그먼트) + 라인·마커·표지판 분류 + 주행 + 임무(mission_node). 브리지는 mission 연동 모드.
노트북 앱 사용자 탭에서 "초기 설정 시작" → 매핑이 끝나면 원·사각 개수를 넣고 "작업 시작".

    ros2 launch agv_bringup agv.launch.py
    ros2 launch agv_bringup agv.launch.py use_fake_hw:=true          # 하드웨어 없이 (카메라는 실제)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("agv_bringup")
    params = os.path.join(share, "config", "params.yaml")

    def node(package, executable):
        return Node(package=package, executable=executable, name=executable, output="screen", parameters=[params])

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, "launch", "s0_hw.launch.py")),
                                 launch_arguments={"standalone": "false"}.items()),
        node("agv_vision", "line_node"),
        node("agv_vision", "marker_node"),
        node("agv_vision", "classifier_node"),
        node("agv_control", "drive_node"),
        node("agv_control", "mission_node"),
    ])

"""S1 라인 주행 — S0(하드웨어 노드 + 블루투스 브리지 + 카메라)에 line_node·drive_node 추가.
노트북 앱 개발자 탭 "측정값 받기 → LINE"으로 오프셋·이벤트 확인, drive_node는 명령을 받을 때만 움직인다.

    ros2 launch agv_bringup s1_line.launch.py
    ros2 launch agv_bringup s1_line.launch.py use_fake_hw:=true     # 하드웨어 없이 (카메라는 실제)
    ros2 action send_goal /drive agv_interfaces/action/Drive "{command: FOLLOW_TO_EVENT}" --feedback   # 다른 터미널
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
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, "launch", "s0_hw.launch.py"))),
        Node(package="agv_vision", executable="line_node", name="line_node", output="screen", parameters=[params]),
        Node(package="agv_control", executable="drive_node", name="drive_node", output="screen", parameters=[params]),
    ])

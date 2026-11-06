"""S0 기초 동작 — 하드웨어 노드 5개(모터·팬틸트·초음파·LED·7세그먼트) + 블루투스 브리지 (+ 카메라). 노트북 앱 개발자 탭으로 시험한다.

    ros2 launch agv_bringup s0_hw.launch.py                       # 실제 하드웨어
    ros2 launch agv_bringup s0_hw.launch.py use_fake_hw:=true     # 가짜 드라이버 (하드웨어 없이)
    ros2 launch agv_bringup s0_hw.launch.py camera:=false transport:=tcp
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(get_package_share_directory("agv_bringup"), "config", "params.yaml")
    fake = ParameterValue(LaunchConfiguration("use_fake_hw"), value_type=bool)
    transport = LaunchConfiguration("transport")
    standalone = ParameterValue(LaunchConfiguration("standalone"), value_type=bool)

    def hw(executable):
        return Node(package="agv_hw", executable=executable, name=executable, output="screen",
                    parameters=[params, {"use_fake_hw": fake}])

    return LaunchDescription([
        DeclareLaunchArgument("use_fake_hw", default_value="false", description="가짜 드라이버 사용"),
        DeclareLaunchArgument("camera", default_value="true", description="camera_ros 실행"),
        DeclareLaunchArgument("transport", default_value="rfcomm", description="rfcomm / tcp"),
        DeclareLaunchArgument("standalone", default_value="true",
                              description="true = 브리지가 상태 관리(단위 시험), false = mission_node 연동(agv.launch.py)"),
        hw("motor_node"),
        hw("pantilt_node"),
        hw("ultrasonic_node"),
        hw("led_node"),
        hw("segment_node"),
        Node(package="agv_bt", executable="bt_bridge_node", name="bt_bridge_node", output="screen",
             parameters=[params, {"transport": transport, "standalone": standalone}]),
        Node(package="camera_ros", executable="camera_node", name="camera", output="screen",
             parameters=[params], condition=IfCondition(LaunchConfiguration("camera"))),
    ])

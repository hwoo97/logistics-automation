"""pantilt_node — /pantilt/pose(LINE·JUNCTION·SIGN) 또는 /dev/pantilt_angle → 서보, 안정화 후 /pantilt/ready (F2).

팬 정면 자동 맞춤 (2026-11-06): /pantilt/calibrate "AUTO" → LINE 자세에서 /line/state 각도를 보며 팬을 돌려
라인이 수직(90°)인 팬 = 정면을 찾음 (agv_hw.calibration) → cal_file에 저장, 모든 자세의 팬을 정면값으로.
"RESET" → 저장값 지우고 params의 자세 각도로. 진행·결과는 /pantilt/calibration
("RUNNING <팬> <라인 각도>", "DONE <팬>", "FAIL <이유>", "RESET <LINE 팬>", "CANCELLED").
SIGN 자세 틸트 저장 (2026-11-06): "SIGN <틸트>" → cal_file에 저장, SIGN 자세로 움직이고 "SIGN <틸트>" 보고.
앱 개발자 탭에서 틸트 슬라이더로 표지판이 보이는 각도를 찾은 뒤 저장 (빌드 없이). RESET이면 같이 지움.
line_node가 돌고 있어야 함 (s1_line.launch.py 또는 agv.launch.py).
"""
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, String

from agv_hw.calibration import PanCalibrator, load_cal, save_cal
from agv_protocol.types import Pose


class PanTiltNode(Node):
    def __init__(self):
        super().__init__("pantilt_node")
        p = self.declare_parameter
        fake = p("use_fake_hw", False).value
        self.pan_ch = p("pan_channel", 0).value
        self.tilt_ch = p("tilt_channel", 1).value
        self.settle_s = p("settle_s", 0.3).value
        self.base_poses = {pose.value: list(p(f"poses.{pose.value}", default).value)
                           for pose, default in ((Pose.LINE, [70, 130]), (Pose.JUNCTION, [65, 165]), (Pose.SIGN, [65, 165]))}
        self.cal_file = os.path.expanduser(p("cal_file", "~/.agv/pantilt_cal.json").value)
        cal = load_cal(self.cal_file)
        self.front, self.sign_tilt = cal["pan_front"], cal["sign_tilt"]
        bus, address = p("i2c_bus", 1).value, p("address", 0x40).value
        oe, chip = p("oe_pin", 17).value, p("gpio_chip", -1).value
        if fake:
            from agv_hw.drivers.fake import FakeServoDriver
            self.driver = FakeServoDriver()
        else:
            from agv_hw.drivers.servo_pca9685 import Pca9685ServoDriver
            self.driver = Pca9685ServoDriver(bus=bus, address=address,
                                             oe_pin=None if oe < 0 else oe, chip=None if chip < 0 else chip)
        self.ready_pub = self.create_publisher(String, "/pantilt/ready", QoSProfile(depth=10))
        self.cal_pub = self.create_publisher(String, "/pantilt/calibration", QoSProfile(depth=10))
        self.create_subscription(String, "/pantilt/pose", self.on_pose, QoSProfile(depth=10))
        self.create_subscription(Float32MultiArray, "/dev/pantilt_angle", self.on_angle, QoSProfile(depth=10))
        self.create_subscription(String, "/pantilt/calibrate", self.on_calibrate, QoSProfile(depth=10))
        try:
            from agv_interfaces.msg import LineState
            self.create_subscription(LineState, "/line/state", self.on_line,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.has_line = True
        except ImportError:                                  # agv_interfaces를 빌드하지 않은 경우 — 자동 맞춤만 못 씀
            self.has_line = False
        self.cal = PanCalibrator()
        self.create_timer(0.2, self.on_cal_tick)
        self._settle_timer = None
        self.pan = self.tilt = 0.0
        self.move(*self.poses[Pose.LINE.value], name=Pose.LINE.value)
        front = f"정면 팬 {self.front}° ({self.cal_file})" if self.front is not None else "정면 맞춤 없음 — params 각도"
        self.get_logger().info(f"pantilt_node 시작 (fake={fake}, 팬 ch{self.pan_ch}, 틸트 ch{self.tilt_ch}, "
                               f"{self.poses}, {front})")

    @property
    def poses(self) -> dict:
        """자세별 [팬, 틸트]. 정면 맞춤값이 있으면 모든 자세의 팬 = 정면, SIGN 틸트 저장값이 있으면 SIGN 틸트 = 저장값."""
        out = {name: [self.front if self.front is not None else pan, tilt] for name, (pan, tilt) in self.base_poses.items()}
        if self.sign_tilt is not None:
            out[Pose.SIGN.value][1] = self.sign_tilt
        return out

    def _save(self):
        try:
            save_cal(self.cal_file, self.front, self.sign_tilt)
        except OSError as e:
            self.get_logger().error(f"팬틸트 저장 실패: {e}")

    def move(self, pan, tilt, name):
        self.pan, self.tilt = float(pan), float(tilt)
        self.driver.set_angle(self.pan_ch, float(pan))
        self.driver.set_angle(self.tilt_ch, float(tilt))
        if self._settle_timer:
            self.destroy_timer(self._settle_timer)
        self._settle_timer = self.create_timer(self.settle_s, lambda: self._ready(name))

    def _ready(self, name):
        self.destroy_timer(self._settle_timer)      # 한 번만 발행
        self._settle_timer = None
        self.ready_pub.publish(String(data=name))

    def on_pose(self, msg: String):
        if msg.data not in self.poses:
            self.get_logger().warn(f"알 수 없는 자세: {msg.data}")
            return
        self._cancel_cal()
        self.move(*self.poses[msg.data], name=msg.data)

    def on_angle(self, msg: Float32MultiArray):
        if len(msg.data) != 2:
            self.get_logger().warn("/dev/pantilt_angle 값 2개(팬, 틸트) 필요")
            return
        self._cancel_cal()
        pan, tilt = (max(0.0, min(180.0, v)) for v in msg.data)
        self.move(pan, tilt, name=f"ANGLE {pan:.0f} {tilt:.0f}")

    # ── 정면 자동 맞춤 ──────────────────────────────────

    def _report(self, text: str):
        self.cal_pub.publish(String(data=text))
        self.get_logger().info(f"정면 맞춤: {text}")

    def _cancel_cal(self):
        if self.cal.active:
            self.cal.cancel()
            self._report("CANCELLED")

    def on_calibrate(self, msg: String):
        parts = msg.data.strip().upper().split()
        cmd = parts[0] if parts else ""
        if cmd == "SIGN" and len(parts) == 2:
            try:
                tilt = max(0.0, min(180.0, float(parts[1])))
            except ValueError:
                self.get_logger().warn(f"/pantilt/calibrate SIGN 각도 오류 ({msg.data!r})")
                return
            self._cancel_cal()
            self.sign_tilt = tilt
            self._save()
            self.move(*self.poses[Pose.SIGN.value], name=Pose.SIGN.value)
            self._report(f"SIGN {tilt:.0f}")
            return
        if cmd == "AUTO":
            if not self.has_line:
                self._report("FAIL agv_interfaces 없음 — 라인 상태를 받을 수 없음")
                return
            self.cal.cancel()
            start_pan, tilt = self.poses[Pose.LINE.value]
            self.cal_tilt = tilt
            step = self.cal.start(start_pan, time.monotonic())
            self.move(step.pan, tilt, name="CAL")
            self.last_line = time.monotonic()
            self._report(f"RUNNING {step.pan:.1f} -")
        elif cmd == "RESET":
            self._cancel_cal()
            self.front = self.sign_tilt = None
            try:
                os.remove(self.cal_file)
            except OSError:
                pass
            self.move(*self.poses[Pose.LINE.value], name=Pose.LINE.value)
            self._report(f"RESET {self.poses[Pose.LINE.value][0]:.1f}")
        else:
            self.get_logger().warn(f"/pantilt/calibrate: AUTO, RESET 또는 SIGN <틸트> ({msg.data!r})")

    def on_line(self, msg):
        if not self.cal.active:
            return
        now = time.monotonic()
        self.last_line = now
        for step in self.cal.feed(float(msg.angle), bool(msg.line_found and msg.angle_valid), now):
            self._apply(step)

    def on_cal_tick(self):
        if not self.cal.active:
            return
        now = time.monotonic()
        if now - self.last_line > 2.0:
            self.cal.cancel()
            self._apply_fail("라인 상태가 안 옴 — line_node 실행 확인 (s1_line.launch.py 또는 agv.launch.py)")
            return
        step = self.cal.tick(now)
        if step:
            self._apply(step)

    def _apply(self, step):
        if step.action == "MOVE":
            self.move(step.pan, self.cal_tilt, name="CAL")
        elif step.action == "MEASURED":
            self._report(f"RUNNING {step.pan:.1f} {step.angle:.1f}")
        elif step.action == "DONE":
            self.front = step.pan
            self._save()
            self.move(*self.poses[Pose.LINE.value], name=Pose.LINE.value)
            self._report(f"DONE {step.pan:.1f}")
        elif step.action == "FAIL":
            self._apply_fail(step.reason)

    def _apply_fail(self, reason: str):
        self.move(*self.poses[Pose.LINE.value], name=Pose.LINE.value)     # 원래 자세로 (저장값은 그대로)
        self._report(f"FAIL {reason}")

    def destroy_node(self):
        self.driver.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PanTiltNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

"""pantilt_node — /pantilt/pose(LINE·JUNCTION·SIGN) 또는 /dev/pantilt_angle → 서보, 안정화 후 /pantilt/ready (F2)."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray, String

from agv_protocol.types import Pose


class PanTiltNode(Node):
    def __init__(self):
        super().__init__("pantilt_node")
        p = self.declare_parameter
        fake = p("use_fake_hw", False).value
        self.pan_ch = p("pan_channel", 0).value
        self.tilt_ch = p("tilt_channel", 1).value
        self.settle_s = p("settle_s", 0.3).value
        self.poses = {pose.value: p(f"poses.{pose.value}", default).value
                      for pose, default in ((Pose.LINE, [70, 130]), (Pose.JUNCTION, [65, 165]), (Pose.SIGN, [65, 180]))}
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
        self.create_subscription(String, "/pantilt/pose", self.on_pose, QoSProfile(depth=10))
        self.create_subscription(Float32MultiArray, "/dev/pantilt_angle", self.on_angle, QoSProfile(depth=10))
        self._settle_timer = None
        self.move(*self.poses[Pose.LINE.value], name=Pose.LINE.value)
        self.get_logger().info(f"pantilt_node 시작 (fake={fake}, 팬 ch{self.pan_ch}, 틸트 ch{self.tilt_ch}, {self.poses})")

    def move(self, pan, tilt, name):
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
        self.move(*self.poses[msg.data], name=msg.data)

    def on_angle(self, msg: Float32MultiArray):
        if len(msg.data) != 2:
            self.get_logger().warn("/dev/pantilt_angle 값 2개(팬, 틸트) 필요")
            return
        pan, tilt = (max(0.0, min(180.0, v)) for v in msg.data)
        self.move(pan, tilt, name=f"ANGLE {pan:.0f} {tilt:.0f}")

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

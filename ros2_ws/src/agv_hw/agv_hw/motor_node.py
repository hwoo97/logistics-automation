"""motor_node — /cmd_vel·/dev/wheels → 바퀴 4개, /safety/estop 래치, 0.5초 워치독 (functional_design F3)."""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool, Float32MultiArray

from agv_hw.drivers.base import BOARD_WHEELS
from agv_hw.logic import Watchdog, mix, wheel_speeds


class MotorNode(Node):
    def __init__(self):
        super().__init__("motor_node")
        p = self.declare_parameter
        fake = p("use_fake_hw", False).value
        chip = p("gpio_chip", -1).value
        duty_max = p("duty_max", 0.75).value
        pwm_hz = p("pwm_hz", 1000).value
        names = p("wheel_map", ["front_left", "front_right", "rear_left", "rear_right"]).value  # L1~L4 순서
        invert = p("invert", [True, False, True, False]).value
        self.turn_gain = p("turn_gain", 1.0).value
        timeout = p("watchdog_s", 0.5).value

        self.wheel_map = dict(zip(BOARD_WHEELS, names))
        if fake:
            from agv_hw.drivers.fake import FakeMotorDriver
            self.driver = FakeMotorDriver()
        else:
            from agv_hw.drivers.motor import LgpioMotorDriver
            self.driver = LgpioMotorDriver(duty_max=duty_max, pwm_hz=pwm_hz,
                                           invert=dict(zip(BOARD_WHEELS, invert)),
                                           chip=None if chip < 0 else chip)
        self.estop = False
        self.moving = False
        self.dog = Watchdog(timeout)

        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, QoSProfile(depth=1))
        self.create_subscription(Float32MultiArray, "/dev/wheels", self.on_dev_wheels, QoSProfile(depth=1))
        self.create_subscription(Bool, "/safety/estop", self.on_estop, QoSProfile(depth=10))
        self.create_timer(0.1, self.on_watchdog)
        self.get_logger().info(f"motor_node 시작 (fake={fake}, duty_max={duty_max}, map={self.wheel_map})")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _apply(self, speeds: dict):
        if self.estop:
            return
        for wheel, speed in speeds.items():
            self.driver.set_wheel(wheel, speed)
        self.moving = any(abs(s) > 1e-3 for s in speeds.values())
        self.dog.feed(self._now())

    def on_cmd_vel(self, msg: Twist):
        left, right = mix(msg.linear.x, msg.angular.z, self.turn_gain)
        self._apply(wheel_speeds(left, right, self.wheel_map))

    def on_dev_wheels(self, msg: Float32MultiArray):
        """개발자 탭: L1~L4 원래 출력 순서 그대로 (바퀴 위치·방향 확인용, invert만 적용)."""
        if len(msg.data) != 4:
            self.get_logger().warn(f"/dev/wheels 값 4개 필요 (받음 {len(msg.data)})")
            return
        self._apply(dict(zip(BOARD_WHEELS, msg.data)))

    def on_estop(self, msg: Bool):
        if msg.data and not self.estop:
            self.get_logger().warn("비상 정지 래치")
        elif not msg.data and self.estop:
            self.get_logger().info("비상 정지 해제")
        self.estop = msg.data
        if self.estop:
            self.driver.stop_all()
            self.moving = False

    def on_watchdog(self):
        if self.moving and self.dog.expired(self._now()):
            self.driver.stop_all()
            self.moving = False
            self.get_logger().info("명령 끊김 → 정지 (워치독)")

    def destroy_node(self):
        self.driver.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

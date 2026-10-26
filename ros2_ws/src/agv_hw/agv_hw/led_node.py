"""led_node — /led/state(OFF·RED·GREEN·ERROR) → LED 2개, ERROR는 0.5초 간격 깜빡임 (F9)."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

from agv_hw.logic import led_output
from agv_protocol.types import LedState


class LedNode(Node):
    def __init__(self):
        super().__init__("led_node")
        p = self.declare_parameter
        fake = p("use_fake_hw", False).value
        red, green, chip = p("red_pin", 16).value, p("green_pin", 26).value, p("gpio_chip", -1).value
        if fake:
            from agv_hw.drivers.fake import FakeLedDriver
            self.driver = FakeLedDriver()
        else:
            from agv_hw.drivers.led import LgpioLedDriver
            self.driver = LgpioLedDriver(red, green, chip=None if chip < 0 else chip)
        self.state = LedState.OFF.value
        self.blink_on = False
        self.create_subscription(String, "/led/state", self.on_state, QoSProfile(depth=10))
        self.create_timer(0.5, self.on_blink)
        self.driver.set(False, False)
        self.get_logger().info(f"led_node 시작 (fake={fake})")

    def on_state(self, msg: String):
        if msg.data not in LedState.__members__:
            self.get_logger().warn(f"알 수 없는 LED 상태: {msg.data}")
            return
        self.state = msg.data
        self.blink_on = True
        self.driver.set(*led_output(self.state, self.blink_on))

    def on_blink(self):
        if self.state == LedState.ERROR.value:
            self.blink_on = not self.blink_on
            self.driver.set(*led_output(self.state, self.blink_on))

    def destroy_node(self):
        self.driver.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

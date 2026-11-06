"""segment_node — /display/digit(String 한 글자: 0~9, '-', 'E', 빈칸) → 7세그먼트 1자리 (F13).
실은 화물 개수를 mission_node가 보낸다. 빈 차량이면 0 (변경 시나리오 2026-11-03) — 시작할 때도 0."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

from agv_hw.drivers.segment import DEFAULT_PINS
from agv_hw.logic import segment_bits


class SegmentNode(Node):
    def __init__(self):
        super().__init__("segment_node")
        p = self.declare_parameter
        fake = p("use_fake_hw", False).value
        pins = list(p("pins", list(DEFAULT_PINS)).value)
        common_anode, chip = p("common_anode", False).value, p("gpio_chip", -1).value
        if fake:
            from agv_hw.drivers.fake import FakeSegmentDriver
            self.driver = FakeSegmentDriver()
        else:
            from agv_hw.drivers.segment import LgpioSegmentDriver
            self.driver = LgpioSegmentDriver(pins, common_anode, chip=None if chip < 0 else chip)
        self.create_subscription(String, "/display/digit", self.on_digit, QoSProfile(depth=10))
        self.driver.set(segment_bits("0"))
        self.get_logger().info(f"segment_node 시작 (fake={fake}, 핀 a~g {pins}, 공통 애노드={common_anode})")

    def on_digit(self, msg: String):
        bits = segment_bits(msg.data)
        if bits is None:
            self.get_logger().warn(f"표시할 수 없는 글자: {msg.data!r}")
            return
        self.driver.set(bits)

    def destroy_node(self):
        self.driver.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SegmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

"""ultrasonic_node — 15Hz 측정 → 중앙값 필터 → /ultrasonic/range (F8). 유효값이 없으면 inf."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

from agv_hw.logic import MedianFilter


class UltrasonicNode(Node):
    def __init__(self):
        super().__init__("ultrasonic_node")
        p = self.declare_parameter
        fake = p("use_fake_hw", False).value
        rate = p("rate_hz", 15.0).value
        self.filter = MedianFilter(p("median_window", 5).value, p("min_range", 0.02).value, p("max_range", 2.0).value)
        trig, echo, chip = p("trig_pin", 5).value, p("echo_pin", 6).value, p("gpio_chip", -1).value
        fake_distance = p("fake_distance", 1.0).value
        if fake:
            from agv_hw.drivers.fake import FakeUltrasonicDriver
            self.driver = FakeUltrasonicDriver(distance=fake_distance)
        else:
            from agv_hw.drivers.ultrasonic import LgpioUltrasonicDriver
            self.driver = LgpioUltrasonicDriver(trig, echo, chip=None if chip < 0 else chip)
        self.pub = self.create_publisher(Range, "/ultrasonic/range", qos_profile_sensor_data)
        self.msg = Range(radiation_type=Range.ULTRASOUND, field_of_view=0.26,
                         min_range=self.filter.min_value, max_range=self.filter.max_value)
        self.msg.header.frame_id = "ultrasonic"
        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(f"ultrasonic_node 시작 (fake={fake}, {rate}Hz)")

    def on_timer(self):
        value = self.filter.push(self.driver.measure())
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.range = value if value is not None else math.inf
        self.pub.publish(self.msg)

    def destroy_node(self):
        self.driver.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

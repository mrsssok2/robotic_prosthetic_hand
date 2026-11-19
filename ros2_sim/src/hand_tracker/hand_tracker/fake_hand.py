"""Fake input: publishes synthetic landmarks (animated finger wave, or fixed curls)."""
import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

from hand_interfaces.msg import HandLandmarks
from hand_tracker.core.synthetic import make_hand

# per-finger flexions at curl = 1 (sum 200 deg for fingers, 90 for the thumb)
_CLOSED = {"little": (60, 80, 60), "index": (60, 80, 60), "middle": (60, 80, 60),
           "ring": (60, 80, 60)}
_ORDER = ("little", "index", "middle", "ring", "thumb")


class FakeHand(Node):
    def __init__(self):
        super().__init__("fake_hand")
        self.declare_parameter("rate", 30.0)
        self.declare_parameter("period", 6.0)
        self.declare_parameter("static_curls", [-1.0])  # 5 values 0..1 -> fixed pose; [-1] = animate
        self.pub = self.create_publisher(HandLandmarks, "/hand/landmarks", 10)
        self.t0 = self.get_clock().now()
        self.create_timer(1.0 / self.get_parameter("rate").value, self.tick)

    def curls(self):
        static = list(self.get_parameter("static_curls").value)
        if len(static) == 5:
            return static
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        period = self.get_parameter("period").value
        return [0.5 - 0.5 * math.cos(2 * math.pi * (t / period - i * 0.12)) for i in range(5)]

    def tick(self):
        flex = {n: (c if n == "thumb" else tuple(a * c for a in _CLOSED[n])) for n, c in zip(_ORDER, self.curls())}
        msg = HandLandmarks()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.handedness, msg.score = "Right", 1.0
        msg.landmarks = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in make_hand(flex)]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeHand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()

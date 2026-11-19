"""HandLandmarks -> HandState (per-finger curl, thumb opposition, gesture)."""
import numpy as np
import rclpy
from rclpy.node import Node

from hand_interfaces.msg import HandLandmarks, HandState
from hand_tracker.core import classify_gesture, finger_curls


class TrackerNode(Node):
    def __init__(self):
        super().__init__("hand_tracker")
        self.declare_parameter("timeout", 0.5)  # s without landmarks -> valid=false
        self.timeout = self.get_parameter("timeout").value
        self.pub = self.create_publisher(HandState, "/hand/state", 10)
        self.create_subscription(HandLandmarks, "/hand/landmarks", self.on_landmarks, 10)
        self.create_timer(0.1, self.watchdog)
        self.last = HandState()
        self.last_rx = None

    def on_landmarks(self, msg: HandLandmarks):
        pts = np.array([[p.x, p.y, p.z] for p in msg.landmarks])
        curl = finger_curls(pts)
        out = HandState()
        out.header = msg.header  # keep the capture stamp (latency tooling later)
        out.valid = True
        out.handedness = msg.handedness
        out.curl = [float(c) for c in curl]
        out.gesture = classify_gesture(curl)
        self.last, self.last_rx = out, self.get_clock().now()
        self.pub.publish(out)

    def watchdog(self):
        if self.last_rx is None or not self.last.valid:
            return
        if (self.get_clock().now() - self.last_rx).nanoseconds * 1e-9 > self.timeout:
            self.last.valid = False
            self.last.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.last)


def main():
    rclpy.init()
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()

"""HandLandmarks -> HandState (per-finger curl, thumb opposition, gesture)."""
import numpy as np
import rclpy
from rclpy.node import Node

from hand_interfaces.msg import HandLandmarks, HandState
from hand_tracker.core import classify_gesture, finger_curls, wrist_angles


class TrackerNode(Node):
    def __init__(self):
        super().__init__("hand_tracker")
        self.declare_parameter("timeout", 0.5)  # s without landmarks -> valid=false
        self.declare_parameter("mirrored", True)  # the landmark source mirrors the image (selfie view)
        self.declare_parameter("wrist", True)     # derive the wrist angles (needs camera-frame landmarks; the synthetic hand has none)
        self.timeout = self.get_parameter("timeout").value
        self.mirrored = self.get_parameter("mirrored").value
        self.use_wrist = self.get_parameter("wrist").value
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
        if self.use_wrist:
            out.wrist = [float(a) for a in wrist_angles(pts, msg.handedness, mirrored=self.mirrored)]
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

"""UDP bridge: receives finger values from the Windows script (windows/hand_cam.py) -> /hand/state.

Packet (JSON): {"t": epoch_s, "hand": "Right", "gesture": "Fist", "curl": [little, index, middle, ring, thumb],
                "wrist": [pronation, flexion], "reach": [x, y, closeness]}
               (wrist: radians, optional; missing = neutral. reach: -1..1, optional, only used in the grasp world)
               {"t": epoch_s, "hand": null}  (no hand)
If nothing valid arrives for `timeout` seconds, a valid=false state is published once.
"""
import json
import math
import socket
import threading

import rclpy
from rclpy.node import Node

from hand_interfaces.msg import HandState


def parse_packet(raw):
    """Return (handedness, gesture, [5 curls], [pronation, flexion], reach or None) or None for 'no hand' / malformed packets.
    reach = [x, y, closeness] in -1..1 (grasp-world reach control), None when the packet has none."""
    try:
        data = json.loads(raw)
        if not data.get("hand"):
            return None
        curl = [min(1.0, max(0.0, float(c))) for c in data["curl"]]
        if len(curl) != 5:
            return None
        wrist = [float(w) for w in data.get("wrist", (0.0, 0.0))]
        if len(wrist) != 2 or not all(map(math.isfinite, wrist)):
            wrist = [0.0, 0.0]
        reach = data.get("reach")
        if reach is not None:
            reach = [max(-1.0, min(1.0, float(r))) for r in reach]
            if len(reach) != 3 or not all(map(math.isfinite, reach)):
                reach = None
        return str(data["hand"]), str(data.get("gesture", "None")), curl, wrist, reach
    except (ValueError, TypeError, KeyError, AttributeError):
        return None


class UdpBridge(Node):
    def __init__(self):
        super().__init__("udp_bridge")
        self.declare_parameter("port", 9870)
        self.declare_parameter("timeout", 0.5)
        self.pub = self.create_publisher(HandState, "/hand/state", 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.get_parameter("port").value))
        self.last = HandState()
        self.last_rx = None
        self.create_timer(0.1, self.watchdog)
        threading.Thread(target=self.receive, daemon=True).start()
        self.get_logger().info(f"listening on udp/{self.get_parameter('port').value}")

    def receive(self):
        while rclpy.ok():
            raw, _ = self.sock.recvfrom(4096)
            parsed = parse_packet(raw)
            if parsed is None:
                continue
            msg = HandState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera"
            msg.valid = True
            msg.handedness, msg.gesture, msg.curl, msg.wrist, reach = parsed
            msg.reach_valid = reach is not None
            msg.reach = reach or [0.0, 0.0, 0.0]
            self.last, self.last_rx = msg, self.get_clock().now()
            self.pub.publish(msg)

    def watchdog(self):
        if self.last_rx is None or not self.last.valid:
            return
        if (self.get_clock().now() - self.last_rx).nanoseconds * 1e-9 > self.get_parameter("timeout").value:
            self.last.valid = False
            self.last.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.last)


def main():
    rclpy.init()
    node = UdpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()

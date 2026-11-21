#!/usr/bin/env python3
"""Latency from a UDP finger packet to the moving Gazebo hand (run in the container while
`ros2 launch hand_bringup hand.launch.py input:=udp gui:=false` is up).

Steps per trial: hold the hand open, then send ONE 'all fingers closed' packet (what hand_cam.py sends when a hand closes)
and time, on the same wall clock:
    bridge      UDP packet -> /hand/state published by udp_bridge
    command     -> first changed /finger_position_controller/commands (controller_node shaping: EMA + rate limit)
    50% / 90%   -> index MCP joint position in Gazebo reaches 50% / 90% of its target

The tracker's own processing time and the camera's capture latency are NOT included here; they are added per tracker in
docs/results (tracker latency from tracking/evaluate.py).
"""
import argparse
import json
import socket
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from hand_interfaces.msg import HandState

TARGET_MCP = 1.57


class Probe(Node):
    def __init__(self):
        super().__init__("latency_probe")
        self.t = {}
        self.create_subscription(HandState, "/hand/state", lambda m: "start" in self.t and self.t.setdefault("bridge", time.time()), 10)
        self.create_subscription(Float64MultiArray, "/finger_position_controller/commands", self.on_cmd, 10)
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.base_cmd = 0.0
        self.mcp = 0.0

    def on_cmd(self, m):
        self.mcp_cmd = m.data[3]                       # index_mcp
        if "start" in self.t and self.mcp_cmd - self.base_cmd > 1e-3:
            self.t.setdefault("command", time.time())

    def on_js(self, m):
        if "index_mcp" not in m.name:
            return
        self.mcp = m.position[m.name.index("index_mcp")]
        now = time.time()
        if "start" in self.t:
            if self.mcp > 0.5 * TARGET_MCP:
                self.t.setdefault("p50", now)
            if self.mcp > 0.9 * TARGET_MCP:
                self.t.setdefault("p90", now)


def spin(node, seconds, until=None):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.005)
        if until and until():
            return True
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=15)
    ap.add_argument("--port", type=int, default=9870)
    ap.add_argument("--out", default="latency.json")
    a = ap.parse_args()
    rclpy.init()
    n = Probe()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(curl):
        sock.sendto(json.dumps({"t": time.time(), "hand": "Right", "gesture": "None", "curl": curl}).encode(), ("127.0.0.1", a.port))

    rows = []
    for i in range(a.trials):
        n.t = {}
        for _ in range(20):                           # open and settled
            send([0.0] * 5); spin(n, 0.05)
        spin(n, 0.5)
        n.base_cmd = getattr(n, "mcp_cmd", 0.0)
        n.t["start"] = time.time()
        send([1.0] * 5)
        for _ in range(60):                           # keep sending like a camera would (30 Hz)
            send([1.0] * 5)
            if spin(n, 0.033, until=lambda: "p90" in n.t):
                break
        if all(k in n.t for k in ("bridge", "command", "p50", "p90")):
            rows.append({k: 1000 * (n.t[k] - n.t["start"]) for k in ("bridge", "command", "p50", "p90")})
    arr = {k: np.array([r[k] for r in rows]) for k in ("bridge", "command", "p50", "p90")}
    out = {"trials": len(rows)}
    for k, v in arr.items():
        out[k] = dict(median_ms=float(np.median(v)), p95_ms=float(np.percentile(v, 95)), min_ms=float(v.min()), max_ms=float(v.max()))
    json.dump(out, open(a.out, "w"), indent=1)
    print(json.dumps(out, indent=1))
    rclpy.shutdown()


if __name__ == "__main__":
    main()

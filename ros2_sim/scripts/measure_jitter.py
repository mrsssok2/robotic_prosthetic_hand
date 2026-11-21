#!/usr/bin/env python3
"""Jitter and tracking lag of the controller's smoothing (run in the container while
`ros2 launch hand_bringup hand.launch.py input:=udp gui:=false` is up; needs no Gazebo physics, it reads the commands).

A fake camera sends UDP finger packets at 30 Hz (like hand_cam.py) and the 50 Hz joint command of the index MCP
(/finger_position_controller/commands[3] / MCP_MAX = curl units) is recorded:

  static    curl held at 0.5 + N(0, sigma) per frame (tracker noise)   -> jitter = std of the command (curl units),
                                                                          noise_gain = jitter / sigma
  tracking  curl = 0.5 + 0.35 sin(2 pi f t) + the same noise, f in 0.5, 1.0, 2.0 Hz (a finger opening/closing)
                                                                       -> lag = delay at the best cross-correlation between
                                                                          the command and the noise-free signal,
                                                                          rmse_aligned = command vs the noise-free signal after removing
                                                                          the lag (curl units; shape + noise error), gain = amplitude
                                                                          of the command relative to the signal (1 = no attenuation)

Same noise sequence (seed) for every run, so before/after runs are comparable.

    python3 scripts/measure_jitter.py --label one_euro --out docs/results/jitter_one_euro.json
"""
import argparse
import json
import socket
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

MCP_MAX = 1.57
RATE = 30.0


class Rec(Node):
    def __init__(self):
        super().__init__("jitter_probe")
        self.rows = []
        self.create_subscription(Float64MultiArray, "/finger_position_controller/commands",
                                 lambda m: self.rows.append((time.time(), m.data[3] / MCP_MAX)), 10)


def run(node, sock, port, signal, seconds):
    """Send signal(t) at RATE Hz for `seconds`; returns (t_send, sent, t_cmd, cmd)."""
    node.rows.clear()
    sent = []
    t0 = time.time()
    k = 0
    while (t := time.time() - t0) < seconds:
        x = signal(t)
        sent.append((t0 + t, x))
        sock.sendto(json.dumps({"t": time.time(), "hand": "Right", "gesture": "None", "curl": [0.0, float(np.clip(x, 0, 1)), 0, 0, 0]}).encode(),
                    ("127.0.0.1", port))
        k += 1
        end = t0 + k / RATE
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.002)
    s, c = np.array(sent), np.array(node.rows)
    return s[:, 0] - t0, s[:, 1], c[:, 0] - t0, c[:, 1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="run")
    ap.add_argument("--sigma", type=float, default=0.03, help="tracker noise per frame (curl units)")
    ap.add_argument("--port", type=int, default=9870)
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--out", default="jitter.json")
    a = ap.parse_args()
    rclpy.init()
    node = Rec()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rng = np.random.default_rng(0)
    noise = rng.normal(0.0, a.sigma, int(a.seconds * RATE * 2))

    def noisy(clean):
        return lambda t: clean(t) + noise[int(t * RATE) % len(noise)]

    out = {"label": a.label, "sigma": a.sigma, "input_rate_hz": RATE}
    for _ in range(40):                                 # settle at 0.5
        sock.sendto(json.dumps({"t": time.time(), "hand": "Right", "gesture": "None", "curl": [0, 0.5, 0, 0, 0]}).encode(), ("127.0.0.1", a.port))
        rclpy.spin_once(node, timeout_sec=0.03)

    _, _, tc, c = run(node, sock, a.port, noisy(lambda t: 0.5), a.seconds)
    c = c[tc > 2.0]
    out["static"] = dict(jitter_std=float(c.std()), noise_gain=float(c.std() / a.sigma),
                         mean_abs_step=float(np.abs(np.diff(c)).mean()), peak_to_peak=float(np.ptp(c)))

    out["tracking"] = {}
    for f in (0.5, 1.0, 2.0):
        clean = lambda t, f=f: 0.5 + 0.35 * np.sin(2 * np.pi * f * t)
        ts, x, tc, c = run(node, sock, a.port, noisy(clean), a.seconds)
        t = np.arange(2.0, a.seconds - 0.5, 0.005)               # skip the start-up, resample on a fine grid
        ref = clean(t)
        cmd = lambda d: np.interp(t + d, tc, c)
        delays = np.arange(0.0, 0.40, 0.005)
        err = [np.sqrt(np.mean((cmd(d) - ref) ** 2)) for d in delays]
        i = int(np.argmin(err))
        out["tracking"][f"{f}Hz"] = dict(lag_ms=float(delays[i] * 1000), rmse_aligned=float(err[i]),
                                         rmse_raw=float(np.sqrt(np.mean((cmd(0.0) - ref) ** 2))),
                                         gain=float(np.dot(cmd(delays[i]) - 0.5, ref - 0.5) / np.dot(ref - 0.5, ref - 0.5)))
    json.dump(out, open(a.out, "w"), indent=1)
    print(json.dumps(out, indent=1))
    rclpy.shutdown()


if __name__ == "__main__":
    main()

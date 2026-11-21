#!/usr/bin/env python3
"""Fake the Windows camera script: send finger values over UDP (stdlib only).

    scripts/fake_udp.py                         # animated: every finger cycles open -> closed
    scripts/fake_udp.py 0 0 0 0 0               # hold a pose (little index middle ring thumb)
    scripts/fake_udp.py 1 1 1 1 1 --seconds 5   # fist for 5 s
    scripts/fake_udp.py 0 0 0 0 0 --wrist 0.8 0.0   # open hand, wrist pronated 0.8 rad (pronation, flexion in rad)
    scripts/fake_udp.py --demo cup              # grasp world (./run.sh grasp:=true auto_grip:=true): reach over the cup, fist, lift, open
    scripts/fake_udp.py --reach 0.5 0.2 0.0     # hold a reach command (x right+, y up+, closeness+; -1..1) with the curls
In the animated mode the wrist also sweeps slowly (pronation and flexion); --no-wrist keeps it neutral.
"""
import argparse
import json
import math
import socket
import time

ap = argparse.ArgumentParser()
ap.add_argument("curl", nargs="*", type=float, help="5 values 0..1; omit to animate")
ap.add_argument("--wrist", nargs=2, type=float, metavar=("PRON", "FLEX"), help="hold the wrist at these angles (rad)")
ap.add_argument("--reach", nargs=3, type=float, metavar=("X", "Y", "CLOSE"), help="send this reach command (-1..1) with every packet")
ap.add_argument("--demo", choices=("cup", "ball", "pen"), help="scripted reach + fist + lift + open on that object (grasp world, ~16 s)")
ap.add_argument("--no-wrist", action="store_true", help="animated mode: keep the wrist neutral")
ap.add_argument("--host", default="127.0.0.1")
ap.add_argument("--port", type=int, default=9870)
ap.add_argument("--rate", type=float, default=30.0)
ap.add_argument("--period", type=float, default=6.0)
ap.add_argument("--seconds", type=float, default=0.0, help="0 = forever")
a = ap.parse_args()
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --demo keyframes: (time s, x, y, closeness, curl). Reach numbers = the palm position over the table (reach_x, reach_y, lift)
# mapped back through the workspace (home 0 / -0.10 / 0.06 m, half range 0.35 / 0.15 / 0.09 m, see hand_controller/shaping.py):
# the object sits 0.05 m right of and 0.11 m in front of the palm centre (the scanned grasp spots of scripts/grasp_test.py).
OBJ_XY = {"cup": (0.0, -0.25), "ball": (0.30, -0.15), "pen": (-0.30, -0.15)}
SPOT = {"cup": (0.05, -0.11), "ball": (0.05, -0.11), "pen": (0.04, -0.085)}


def demo_keys(name):
    px, py = OBJ_XY[name][0] - SPOT[name][0], OBJ_XY[name][1] - SPOT[name][1]
    x, y = px / 0.35, (-0.10 - py) / 0.15
    down, up, home = (0.06 - 0.0) / 0.09, (0.06 - 0.10) / 0.09, 0.0       # lift 0 (grasp height), 0.10 (lifted), 0.06 (home)
    o, f = [0.0] * 5, [1.0] * 5
    return [(0.0, 0, 0, home, o), (2.0, 0, 0, home, o), (5.0, x, y, home, o), (7.0, x, y, down, o),   # reach over it, lower
            (8.0, x, y, down, o), (8.4, x, y, down, f), (12.0, x, y, down, f),                       # close (auto grip) and settle
            (14.0, x, y, up, f), (16.0, x, y, up, f), (16.4, x, y, up, o), (18.0, x, y, up, o)]      # lift, hold, open


def lerp_keys(keys, t):
    for (t0, *a), (t1, *b) in zip(keys, keys[1:]):
        if t0 <= t <= t1:
            k = (t - t0) / (t1 - t0)
            return [a[i] + k * (b[i] - a[i]) for i in range(3)], [a[3][i] + k * (b[3][i] - a[3][i]) for i in range(5)]
    return keys[-1][1:4], keys[-1][4]


def gesture_of(curl):
    if all(c > 0.6 for c in curl[:4]):
        return "Fist"
    return "Open Hand" if all(c < 0.35 for c in curl) else "None"


keys = demo_keys(a.demo) if a.demo else None
t0 = time.time()
if keys:
    a.seconds = keys[-1][0] + 1.0
while not a.seconds or time.time() - t0 < a.seconds:
    t = time.time() - t0
    curl = a.curl if len(a.curl) == 5 else [0.5 - 0.5 * math.cos(2 * math.pi * (t / a.period - i * 0.1)) for i in range(5)]
    reach = list(a.reach) if a.reach else None
    if keys:
        reach, curl = lerp_keys(keys, t)
    animated = len(a.curl) != 5 and not keys
    wrist = a.wrist or ([0.8 * math.sin(2 * math.pi * t / (2 * a.period)), 0.6 * math.sin(2 * math.pi * t / (1.5 * a.period))]
                        if animated and not a.no_wrist else [0.0, 0.0])
    pkt = {"t": time.time(), "hand": "Right", "gesture": gesture_of(curl) if keys or reach else "Fake", "curl": curl, "wrist": wrist}
    if reach is not None:
        pkt["reach"] = reach
    sock.sendto(json.dumps(pkt).encode(), (a.host, a.port))
    time.sleep(1 / a.rate)

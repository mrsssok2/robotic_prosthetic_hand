#!/usr/bin/env python3
"""Fake the Windows camera script: send finger values over UDP (stdlib only).

    scripts/fake_udp.py                         # animated: every finger cycles open -> closed
    scripts/fake_udp.py 0 0 0 0 0               # hold a pose (little index middle ring thumb)
    scripts/fake_udp.py 1 1 1 1 1 --seconds 5   # fist for 5 s
"""
import argparse
import json
import math
import socket
import time

ap = argparse.ArgumentParser()
ap.add_argument("curl", nargs="*", type=float, help="5 values 0..1; omit to animate")
ap.add_argument("--host", default="127.0.0.1")
ap.add_argument("--port", type=int, default=9870)
ap.add_argument("--rate", type=float, default=30.0)
ap.add_argument("--period", type=float, default=6.0)
ap.add_argument("--seconds", type=float, default=0.0, help="0 = forever")
a = ap.parse_args()
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
t0 = time.time()
while not a.seconds or time.time() - t0 < a.seconds:
    t = time.time() - t0
    curl = a.curl if len(a.curl) == 5 else [0.5 - 0.5 * math.cos(2 * math.pi * (t / a.period - i * 0.1)) for i in range(5)]
    pkt = {"t": time.time(), "hand": "Right", "gesture": "Fake", "curl": curl}
    sock.sendto(json.dumps(pkt).encode(), (a.host, a.port))
    time.sleep(1 / a.rate)

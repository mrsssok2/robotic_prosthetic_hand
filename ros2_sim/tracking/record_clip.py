"""Record a webcam clip for evaluate.py --clips (Windows or Linux):  python record_clip.py clips/open_close.mp4 --seconds 20"""
import argparse
import time

import cv2

ap = argparse.ArgumentParser()
ap.add_argument("out")
ap.add_argument("--camera", type=int, default=0)
ap.add_argument("--seconds", type=float, default=20)
a = ap.parse_args()
cap = cv2.VideoCapture(a.camera)
cap.set(3, 640); cap.set(4, 480)
w = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*"mp4v"), 30, (640, 480))
t0 = time.time()
while time.time() - t0 < a.seconds:
    ok, f = cap.read()
    if not ok:
        break
    f = cv2.flip(f, 1)
    w.write(f); cv2.imshow("recording (q stops)", f)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
w.release(); cap.release()

"""Validate the 2-D curl against MediaPipe's 3-D (world-landmark) curl on the same images.

    tracking/run_in_docker.sh python tracking/validate_curl2d.py --images data/val_sel.txt --n 800 --calibrate
    tracking/run_in_docker.sh python tracking/validate_curl2d.py --images data/test.txt --n 1000
Reference = finger_curls(MediaPipe world landmarks) (3-D, metric). Candidates use only the image keypoints (x, y).
"""
import argparse
import pathlib
import sys

import cv2
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "hand_tracker"))
from hand_tracker.core import FINGER_ORDER, curl as curlmod, finger_curls  # noqa: E402
from hand_tracker.core.landmarks import FINGER_CHAINS  # noqa: E402
import mediapipe as mp  # noqa: E402


def collect(paths):
    hands = mp.solutions.hands.Hands(static_image_mode=True, max_num_hands=1)
    rows = []
    for p in paths:
        img = cv2.imread(p)
        r = hands.process(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        if not r.multi_hand_landmarks or not r.multi_hand_world_landmarks:
            continue
        h, w = img.shape[:2]
        xy = np.array([[l.x * w, l.y * h] for l in r.multi_hand_landmarks[0].landmark])
        world = np.array([[l.x, l.y, l.z] for l in r.multi_hand_world_landmarks[0].landmark])
        rows.append((xy, finger_curls(world)))
    return rows


def score(rows, fn):
    err = np.array([np.abs(fn(xy) - ref) for xy, ref in rows])
    ok = np.array([(fn(xy) >= .5) == (ref >= .5) for xy, ref in rows])
    return err.mean(), err.mean(0), ok.mean(), ok.mean(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", default=str(ROOT / "data" / "test.txt"))
    ap.add_argument("--n", type=int, default=1000)
    ap.add_argument("--calibrate", action="store_true")
    a = ap.parse_args()
    paths = [l.strip() for l in open(a.images) if l.strip()][:a.n]
    rows = collect(paths)
    print(len(rows), "images with a MediaPipe hand")
    if a.calibrate:   # per-finger median tip/MCP ratio (relative to wrist) for open (ref<0.15) and closed (ref>0.85) fingers
        for i, name in enumerate(FINGER_ORDER[:4]):
            ch = FINGER_CHAINS[name]
            r = np.array([[np.linalg.norm(xy[ch[-1]] - xy[0]) / np.linalg.norm(xy[ch[1]] - xy[0]), ref[i]] for xy, ref in rows])
            print(name, "open ratio", np.median(r[r[:, 1] < .15, 0]).round(2), "closed ratio", np.median(r[r[:, 1] > .85, 0]).round(2))
    old = lambda xy: finger_curls(np.c_[xy, np.zeros(21)], method="3d")
    new = lambda xy: finger_curls(xy)               # auto -> 2-D method
    print("method | MAE | per finger | open/closed agree | per finger")
    for name, fn in (("2-D angles only (old, z=0)", old), ("2-D angles + tip distance (new)", new)):
        m, mf, o, of = score(rows, fn)
        print(f"{name:34s} {m:.3f} {np.round(mf, 3)} {o:.1%} {np.round(of, 3)}")


if __name__ == "__main__":
    main()

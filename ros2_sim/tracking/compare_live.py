"""Side-by-side YOLO vs MediaPipe on the same frames, with colour-coded bone overlays and agreement numbers.

    python tracking/compare_live.py --source clip.mp4            # video (record with tracking/record_clip.py), already-mirrored frames
    python tracking/compare_live.py --images data/test.txt --scales 1.0,0.5,0.3   # dataset hands pasted into 640x480 frames
    python tracking/compare_live.py --camera 0 --window           # live (Windows), q quits

Left panel = MediaPipe (reference), right = YOLO. Fingers: thumb red, index orange, middle green, ring cyan, little magenta.
Prints per-frame hand-found rates, keypoint error (% of MediaPipe hand-box diagonal), curl MAE and open/closed agreement vs MediaPipe.
"""
import argparse
import pathlib
import sys

import cv2
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "hand_tracker"))
from hand_tracker.core import FINGER_ORDER  # noqa: E402
from hand_tracker.trackers import curl_from_image_keypoints, make_tracker  # noqa: E402

CHAINS = {"thumb": (0, 1, 2, 3, 4), "index": (0, 5, 6, 7, 8), "middle": (0, 9, 10, 11, 12),
          "ring": (0, 13, 14, 15, 16), "little": (0, 17, 18, 19, 20)}
BGR = {"thumb": (60, 60, 255), "index": (0, 150, 255), "middle": (60, 200, 60), "ring": (220, 200, 0), "little": (220, 60, 220)}


def draw_skeleton(img, kp, label):
    out = img.copy()
    if kp is not None:
        for name, chain in CHAINS.items():
            pts = [tuple(int(v) for v in kp[i]) for i in chain]
            for a, b in zip(pts[:-1], pts[1:]):
                cv2.line(out, a, b, BGR[name], 2, cv2.LINE_AA)
            for p in pts[1:]:
                cv2.circle(out, p, 3, BGR[name], -1, cv2.LINE_AA)
    cv2.putText(out, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return out


def paste(img, scale, w=640, h=480):
    canvas = np.full((h, w, 3), 100, np.uint8)
    s = int(h * scale)
    r = s / max(img.shape[:2])
    small = cv2.resize(img, (int(img.shape[1] * r), int(img.shape[0] * r)))
    y0, x0 = (h - small.shape[0]) // 2, (w - small.shape[1]) // 2
    canvas[y0:y0 + small.shape[0], x0:x0 + small.shape[1]] = small
    return canvas


def frames(a):
    if a.images:
        paths = [l.strip() for l in open(a.images) if l.strip()][:a.n]
        for s in [float(v) for v in a.scales.split(",")]:
            for p in paths:
                yield s, paste(cv2.imread(p), s)
    elif a.source:
        cap = cv2.VideoCapture(a.source)
        i = 0
        while True:
            ok, f = cap.read()
            if not ok:
                break
            i += 1
            if i % a.stride == 0:
                yield 1.0, f
    else:
        cap = cv2.VideoCapture(a.camera)
        while True:
            ok, f = cap.read()
            if not ok:
                break
            yield 1.0, cv2.flip(f, 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--onnx", default=str(ROOT / "models" / "hand_pose_n640.onnx"))
    ap.add_argument("--images"); ap.add_argument("--scales", default="1.0,0.6,0.4,0.25"); ap.add_argument("--n", type=int, default=150)
    ap.add_argument("--source"); ap.add_argument("--stride", type=int, default=3)
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--window", action="store_true")
    ap.add_argument("--out", default=str(ROOT / "docs" / "results" / "compare"))
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--save", type=int, default=3, help="overlay images to save per group")
    a = ap.parse_args()
    out = pathlib.Path(a.out); out.mkdir(parents=True, exist_ok=True)
    mp_t = make_tracker("mediapipe", static_image_mode=bool(a.images))
    yo = make_tracker("yolo", model=a.onnx, conf=a.conf, providers=["CPUExecutionProvider"])
    stats, saved = {}, {}
    for scale, f in frames(a):
        rm, ry = mp_t.process(f), yo.process(f)
        s = stats.setdefault(scale, dict(n=0, mp=0, yo=0, both=0, nme=[], curl=[], oc=[]))
        s["n"] += 1; s["mp"] += rm is not None; s["yo"] += ry is not None
        if rm is not None and ry is not None:
            s["both"] += 1
            diag = np.hypot(*np.ptp(rm.keypoints, axis=0))
            s["nme"].append(np.linalg.norm(rm.keypoints - ry.keypoints, axis=1).mean() / max(diag, 1))
            ref = rm.extra["curl_2d"]
            s["curl"].append(np.abs(ry.curl - ref)); s["oc"].append((ry.curl >= .5) == (ref >= .5))
        panel = np.hstack([draw_skeleton(f, None if rm is None else rm.keypoints, "MediaPipe"),
                           draw_skeleton(f, None if ry is None else ry.keypoints, "YOLO" + ("" if ry is None else f" {ry.score:.2f}"))])
        if saved.get(scale, 0) < a.save:
            cv2.imwrite(str(out / f"overlay_s{scale:.2f}_{saved.get(scale, 0)}.jpg"), panel)
            saved[scale] = saved.get(scale, 0) + 1
        if a.window:
            cv2.imshow("MediaPipe | YOLO", panel)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    print("scale | frames | MP found | YOLO found | kp err %diag | curl MAE | open/closed agree")
    for scale, s in stats.items():
        print(f"{scale:5.2f} | {s['n']:6d} | {s['mp']/s['n']:7.0%} | {s['yo']/s['n']:9.0%} | "
              f"{100*np.mean(s['nme']) if s['nme'] else float('nan'):11.1f} | "
              f"{np.mean(s['curl']) if s['curl'] else float('nan'):8.3f} | {np.mean(s['oc']) if s['oc'] else float('nan'):7.1%}")


if __name__ == "__main__":
    main()

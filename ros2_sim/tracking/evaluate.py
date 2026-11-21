"""Compare the three trackers (yolo | mediapipe | classic) on the same inputs.

Accuracy: held-out hand-keypoints images (data/test.txt, never seen in training or checkpoint selection) OR your own
recorded clips (--clips DIR, videos; the reference there is MediaPipe run on the same frames, so it measures agreement
with MediaPipe, not truth).  Speed: FPS + latency of ``tracker.process`` on 640x480 frames.

    tracking/run_in_docker.sh python tracking/evaluate.py --onnx models/hand_pose_n320.onnx --out docs/results

REFERENCE LABELS COME FROM MEDIAPIPE: the Ultralytics hand-keypoints dataset was auto-labelled with MediaPipe Hands.
"Error vs the labels" therefore means "error vs MediaPipe's output on that image", and MediaPipe is evaluated against
labels it (more or less) produced itself, which favours it. No independent human annotation is involved.
"""
import argparse
import json
import pathlib
import sys
import time

import cv2
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "hand_tracker"))
from hand_tracker.core import FINGER_ORDER  # noqa: E402
from hand_tracker.trackers import curl_from_image_keypoints, make_tracker  # noqa: E402

CLOSED = 0.5   # curl >= 0.5 -> "closed"


def read_label(path, w, h):
    rows = [list(map(float, l.split())) for l in pathlib.Path(path).read_text().splitlines() if l.strip()]
    if not rows:
        return None
    r = max(rows, key=lambda r: r[3] * r[4])           # largest hand
    kp = np.array(r[5:5 + 63]).reshape(21, 3)
    xy = kp[:, :2] * [w, h]
    box = np.array(r[1:5]) * [w, h, w, h]
    return xy, float(np.hypot(box[2], box[3]))


def label_path(img):
    return str(img).replace("/images/", "/labels/").rsplit(".", 1)[0] + ".txt"


def eval_images(trackers, images, limit):
    gt_closed = []
    per = {n: dict(found=0, nme=[], pck=[], curl_err=[], open_ok=[], total=0, ref_vs=[]) for n in trackers}
    for k, img_path in enumerate(images[:limit]):
        img = cv2.imread(str(img_path))
        lab = read_label(label_path(img_path), img.shape[1], img.shape[0])
        if lab is None:
            continue
        gt_kp, diag = lab
        gt_curl = curl_from_image_keypoints(gt_kp)
        gt_closed.append(gt_curl >= CLOSED)
        for name, trk in trackers.items():
            s = per[name]
            s["total"] += 1
            res = trk.process(img)
            if res is None:
                s["open_ok"].append(np.zeros(5, bool))   # a miss counts as wrong for open/closed accuracy
                continue
            s["found"] += 1
            if res.keypoints is not None:
                err = np.linalg.norm(res.keypoints - gt_kp, axis=1) / diag
                s["nme"].append(err.mean())
                s["pck"].append((err < 0.05).mean())
            ref = res.extra.get("curl_2d", res.curl) if res.extra else res.curl
            s["curl_err"].append(np.abs(ref - gt_curl))
            s["open_ok"].append((ref >= CLOSED) == (gt_curl >= CLOSED))
        if k % 500 == 0:
            print(f"  {k}/{min(limit, len(images))}", flush=True)
    out = {}
    for name, s in per.items():
        ce = np.array(s["curl_err"]) if s["curl_err"] else np.zeros((0, 5))
        oo = np.array(s["open_ok"]) if s["open_ok"] else np.zeros((0, 5), bool)
        out[name] = dict(
            images=s["total"], detection_rate=s["found"] / max(s["total"], 1),
            nme_pct=100 * float(np.mean(s["nme"])) if s["nme"] else None,
            pck05=float(np.mean(s["pck"])) if s["pck"] else None,
            curl_mae=float(ce.mean()) if len(ce) else None,
            curl_mae_per_finger={f: float(ce[:, i].mean()) for i, f in enumerate(FINGER_ORDER)} if len(ce) else None,
            openclosed_acc=float(oo.mean()) if len(oo) else None,
            openclosed_acc_per_finger={f: float(oo[:, i].mean()) for i, f in enumerate(FINGER_ORDER)} if len(oo) else None)
    p = np.mean(gt_closed, axis=0)
    out['_majority_baseline'] = float(np.mean(np.maximum(p, 1 - p)))
    return out


def webcam_like(img, w=640, h=480, scale=0.55):
    """Hand crop placed on a 640x480 canvas (stand-in for a webcam frame when timing)."""
    canvas = np.full((h, w, 3), 90, np.uint8)
    s = int(h * scale)
    canvas[(h - s) // 2:(h - s) // 2 + s, (w - s) // 2:(w - s) // 2 + s] = cv2.resize(img, (s, s))
    return canvas


def eval_speed(trackers, images, n=300):
    frames = [webcam_like(cv2.imread(str(p))) for p in images[:50]]
    out = {}
    for name, trk in trackers.items():
        for f in frames[:10]:
            trk.process(f)                              # warm-up
        t = []
        for i in range(n):
            t0 = time.perf_counter()
            trk.process(frames[i % len(frames)])
            t.append((time.perf_counter() - t0) * 1000)
        t = np.array(t)
        out[name] = dict(frames=n, fps=1000 / t.mean(), lat_mean_ms=float(t.mean()), lat_p50_ms=float(np.percentile(t, 50)),
                         lat_p95_ms=float(np.percentile(t, 95)))
    return out


def eval_clips(trackers, clips_dir, stride=2):
    """Agreement with MediaPipe on recorded clips (reference = MediaPipe on the same frame, not truth)."""
    ref_trk = make_tracker("mediapipe", static_image_mode=False)
    res_all = {n: dict(found=0, total=0, curl_err=[], open_ok=[]) for n in trackers}
    for clip in sorted(pathlib.Path(clips_dir).glob("*.mp4")):
        cap = cv2.VideoCapture(str(clip))
        i = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            i += 1
            if i % stride:
                continue
            ref = ref_trk.process(frame)
            if ref is None:
                continue
            ref_curl = ref.extra["curl_2d"] if ref.extra and "curl_2d" in ref.extra else ref.curl
            for n, trk in trackers.items():
                s = res_all[n]
                s["total"] += 1
                r = trk.process(frame)
                if r is None:
                    s["open_ok"].append(np.zeros(5, bool)); continue
                s["found"] += 1
                c = r.extra.get("curl_2d", r.curl) if r.extra else r.curl
                s["curl_err"].append(np.abs(c - ref_curl)); s["open_ok"].append((c >= CLOSED) == (ref_curl >= CLOSED))
    out = {}
    for n, s in res_all.items():
        ce, oo = np.array(s["curl_err"]), np.array(s["open_ok"])
        out[n] = dict(frames=s["total"], detection_rate=s["found"] / max(s["total"], 1),
                      curl_mae=float(ce.mean()) if len(ce) else None, openclosed_acc=float(oo.mean()) if len(oo) else None)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--onnx", default=str(ROOT / "models" / "hand_pose_n320.onnx"))
    ap.add_argument("--images", default=str(ROOT / "data" / "test.txt"))
    ap.add_argument("--limit", type=int, default=3000)
    ap.add_argument("--clips", help="directory of recorded .mp4 clips (optional)")
    ap.add_argument("--out", default=str(ROOT / "docs" / "results"))
    ap.add_argument("--no-gpu-onnx", action="store_true")
    a = ap.parse_args()
    out = pathlib.Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    images = [l.strip() for l in open(a.images) if l.strip()]
    trackers = {"yolo": make_tracker("yolo", model=a.onnx, providers=["CPUExecutionProvider"]),
                "mediapipe": make_tracker("mediapipe", static_image_mode=True),
                "classic": make_tracker("classic")}
    print("accuracy on", min(a.limit, len(images)), "held-out images")
    acc = eval_images(trackers, images, a.limit)
    majority = acc.pop('_majority_baseline')
    trackers["mediapipe"] = make_tracker("mediapipe", static_image_mode=False)   # video mode = how hand_cam.py uses it
    speed_trackers = dict(trackers)
    speed = eval_speed({"yolo (onnx cpu)": trackers["yolo"], "mediapipe (cpu)": trackers["mediapipe"], "classic (cpu)": trackers["classic"]}, images)
    import onnxruntime as ort
    if not a.no_gpu_onnx and "CUDAExecutionProvider" in ort.get_available_providers():
        gpu = make_tracker("yolo", model=a.onnx, providers=["CUDAExecutionProvider", "CPUExecutionProvider"])
        speed.update(eval_speed({"yolo (onnx cuda)": gpu}, images))
    result = dict(accuracy=acc, majority_baseline=majority, speed=speed, test_images=min(a.limit, len(images)), onnx=pathlib.Path(a.onnx).name,
                  note="labels are MediaPipe-generated (Ultralytics hand-keypoints); see docs/results/README.md")
    if a.clips:
        result["clips"] = eval_clips({k: v for k, v in trackers.items()}, a.clips)
    (out / "tracker_eval.json").write_text(json.dumps(result, indent=1))
    print(json.dumps(result, indent=1))


if __name__ == "__main__":
    main()

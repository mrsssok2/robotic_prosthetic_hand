"""Self-collected distillation data: auto-label your own webcam clips with MediaPipe (21 keypoints, YOLO pose format).

    tracking/run_in_docker.sh python tracking/make_selfcollected_dataset.py \\
        --clips data/clips/train1.mp4 data/clips/train2.mp4 --orig-train 4000 --repeat 3

Per frame (every --step-th candidate; video mode runs MediaPipe on every frame so tracking works): MediaPipe Hands, keep only confident detections
(handedness score >= --min-score, all 21 keypoints inside the frame, hand not tiny). Near-duplicate frames are dropped:
a frame is kept if the keypoints moved >= --min-motion (fraction of the hand-box diagonal) since the last kept frame,
or if --max-gap seconds passed (so still poses are kept at a low rate). The LAST --val-frac of every clip (minus a
--gap-s gap) is the validation split, so no neighbouring frames leak between train and val.

NEVER uses the test clip: any path whose name contains "test" is refused, and the written manifest lists the exact
source clips. Writes data/self/{train,val}/{images,labels}, data/train_self.txt (self frames repeated --repeat times +
a sample of the original mix), data/val_self.txt, data/hand-self.yaml, data/self/manifest.json.

Honest caveat: labels are MediaPipe's output, so a model trained on them can at best agree with MediaPipe; it is
distillation, not independent ground truth. Frames contain the user's face, so data/ stays gitignored.
"""
import argparse
import json
import pathlib
import random

import cv2
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]


def label_line(kp, w, h, pad=0.12):
    x0, y0, x1, y1 = kp[:, 0].min(), kp[:, 1].min(), kp[:, 0].max(), kp[:, 1].max()
    bw, bh = (x1 - x0) * (1 + 2 * pad), (y1 - y0) * (1 + 2 * pad)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    vals = [0, cx / w, cy / h, min(bw / w, 1), min(bh / h, 1)]
    for x, y in kp:
        vals += [x / w, y / h, 2]
    return " ".join(f"{v:.6f}" if k else "0" for k, v in enumerate(vals))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--clips", nargs="+", required=True)
    ap.add_argument("--step", type=int, default=2)
    ap.add_argument("--mode", choices=("video", "static"), default="video",
                    help="video = MediaPipe tracking mode over every frame (same as the compare_live reference, finds far more hands); static = per-frame detection")
    ap.add_argument("--min-score", type=float, default=0.85)
    ap.add_argument("--min-detect", type=float, default=0.6)
    ap.add_argument("--min-diag", type=float, default=60, help="minimum hand diagonal in px")
    ap.add_argument("--min-motion", type=float, default=0.04)
    ap.add_argument("--max-gap", type=float, default=1.0)
    ap.add_argument("--val-frac", type=float, default=0.2)
    ap.add_argument("--gap-s", type=float, default=2.0)
    ap.add_argument("--orig-train", type=int, default=4000, help="original/composite images mixed into train")
    ap.add_argument("--orig-val", type=int, default=500)
    ap.add_argument("--repeat", type=int, default=3, help="repeat self train frames this many times in the list")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    for c in a.clips:
        if "test" in pathlib.Path(c).name.lower():
            raise SystemExit(f"refusing {c}: test clips must never be used for training")
    import mediapipe as mp
    hands = mp.solutions.hands.Hands(static_image_mode=a.mode == "static", max_num_hands=1,
                           min_detection_confidence=a.min_detect, min_tracking_confidence=0.5)
    out = ROOT / "data" / "self"
    for s in ("train", "val"):
        (out / s / "images").mkdir(parents=True, exist_ok=True); (out / s / "labels").mkdir(exist_ok=True)
        for p in list((out / s / "images").glob("*")) + list((out / s / "labels").glob("*")):
            p.unlink()                                           # rebuild from scratch
    files = {"train": [], "val": []}
    manifest = {"clips": {}, "args": vars(a)}
    for clip in a.clips:
        stem = pathlib.Path(clip).stem
        cap = cv2.VideoCapture(clip)
        n, fps = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)), cap.get(cv2.CAP_PROP_FPS)
        if n <= 0:
            print(f"{clip}: no frames (invalid recording) - skipped"); manifest["clips"][stem] = "invalid"; continue
        val_start = int(n * (1 - a.val_frac))
        gap = int(a.gap_s * fps)
        st = dict(frames=n, candidates=0, detected=0, low_score=0, outside=0, tiny=0, near_dup=0, kept_train=0, kept_val=0)
        last_kp, last_i = None, -10**9
        for i in range(n):
            ok, f = cap.read()
            if not ok:
                break
            skip = i % a.step or val_start - gap <= i < val_start
            if skip and a.mode == "static":
                continue
            if val_start - gap <= i < val_start:
                if i == val_start - gap:
                    hands.reset() if hasattr(hands, "reset") else None
                continue                                         # gap between train and val
            res = hands.process(cv2.cvtColor(f, cv2.COLOR_BGR2RGB))      # video mode needs every frame
            if skip:
                continue
            st["candidates"] += 1
            if not res.multi_hand_landmarks:
                continue
            st["detected"] += 1
            h, w = f.shape[:2]
            if res.multi_handedness[0].classification[0].score < a.min_score:
                st["low_score"] += 1; continue
            kp = np.array([[p.x * w, p.y * h] for p in res.multi_hand_landmarks[0].landmark])
            if (kp[:, 0] < -0.01 * w).any() or (kp[:, 0] > 1.01 * w).any() or (kp[:, 1] < -0.01 * h).any() or (kp[:, 1] > 1.01 * h).any():
                st["outside"] += 1; continue
            kp = np.clip(kp, 0, [w - 1, h - 1])
            diag = float(np.hypot(*np.ptp(kp, axis=0)))
            if diag < a.min_diag:
                st["tiny"] += 1; continue
            if last_kp is not None and i - last_i < a.max_gap * fps and np.linalg.norm(kp - last_kp, axis=1).mean() / diag < a.min_motion:
                st["near_dup"] += 1; continue
            last_kp, last_i = kp, i
            split = "val" if i >= val_start else "train"
            name = f"{stem}_{i:05d}"
            cv2.imwrite(str(out / split / "images" / f"{name}.jpg"), f, [cv2.IMWRITE_JPEG_QUALITY, 92])
            (out / split / "labels" / f"{name}.txt").write_text(label_line(kp, w, h) + "\n")
            files[split].append(str(out / split / "images" / f"{name}.jpg"))
            st[f"kept_{split}"] += 1
        manifest["clips"][stem] = st
        print(stem, st, flush=True)
    rng = random.Random(a.seed)
    orig_t = [l.strip() for l in open(ROOT / "data" / "train_mix.txt") if l.strip()]
    orig_v = [l.strip() for l in open(ROOT / "data" / "val_mix.txt") if l.strip()]
    train = files["train"] * a.repeat + rng.sample(orig_t, min(a.orig_train, len(orig_t)))
    val = files["val"] + rng.sample(orig_v, min(a.orig_val, len(orig_v)))
    rng.shuffle(train)
    (ROOT / "data" / "train_self.txt").write_text("\n".join(train) + "\n")
    (ROOT / "data" / "val_self.txt").write_text("\n".join(val) + "\n")
    (ROOT / "data" / "hand-self.yaml").write_text(f"""path: {ROOT / 'data'}
train: {ROOT / 'data' / 'train_self.txt'}
val: {ROOT / 'data' / 'val_self.txt'}
kpt_shape: [21, 3]
flip_idx: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
names:
  0: hand
""")
    manifest["lists"] = dict(self_train=len(files["train"]), self_val=len(files["val"]), train_list=len(train), val_list=len(val))
    (out / "manifest.json").write_text(json.dumps(manifest, indent=1))
    print(manifest["lists"])


if __name__ == "__main__":
    main()

"""Composite webcam-like training frames from the tight hand crops of the hand-keypoints dataset.

The dataset images are 224 px crops with the hand filling the frame; a webcam frame (640x480) has a small hand on a busy
background, often backlit. Each composite: random background (gradient / noise / blurred other crop), the hand crop scaled to
0.18-0.85 of the frame height at a random position, then random brightness / gamma / backlight gradient / low contrast /
sensor noise. Labels (box + 21 keypoints) are transformed to match.

    tracking/run_in_docker.sh python tracking/make_webcam_dataset.py --n-train 9000 --n-val 1500
Writes data/webcam/{train,val}/images|labels, data/train_mix.txt (composites + 4000 originals), data/val_mix.txt, data/hand-webcam.yaml
"""
import argparse
import pathlib
import random

import cv2
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
W, H = 640, 480


def read(path):
    img = cv2.imread(path)
    lab = pathlib.Path(path.replace("/images/", "/labels/").rsplit(".", 1)[0] + ".txt").read_text().split("\n")
    rows = [list(map(float, l.split())) for l in lab if l.strip()]
    return img, rows


def background(rng, pool):
    k = rng.random()
    if k < 0.3:
        c0, c1 = np.array([rng.randint(30, 230) for _ in range(3)]), np.array([rng.randint(30, 230) for _ in range(3)])
        t = np.linspace(0, 1, W)[None, :, None] if rng.random() < .5 else np.linspace(0, 1, H)[:, None, None]
        bg = (c0 + (c1 - c0) * t).astype(np.uint8) * np.ones((H, W, 3), np.uint8)
        return np.broadcast_to(bg, (H, W, 3)).copy()
    if k < 0.55:
        bg = np.random.default_rng(rng.randint(0, 10**9)).integers(0, 255, (H // 8, W // 8, 3), dtype=np.uint8)
        return cv2.GaussianBlur(cv2.resize(bg, (W, H), interpolation=cv2.INTER_CUBIC), (0, 0), 6)
    img = cv2.imread(rng.choice(pool))            # blurred, enlarged other crop = realistic textures/skin tones
    img = cv2.resize(img, (W, H))
    return cv2.GaussianBlur(img, (0, 0), rng.uniform(5, 14))


def lighting(img, rng):
    f = img.astype(np.float32) / 255
    f = f ** rng.uniform(0.6, 1.8)                                 # gamma
    f = (f - 0.5) * rng.uniform(0.5, 1.2) + 0.5                    # contrast
    f = f * rng.uniform(0.45, 1.25)                                # exposure
    if rng.random() < 0.5:                                         # backlight: bright side / dark foreground gradient
        ang = rng.uniform(0, 2 * np.pi)
        gx, gy = np.meshgrid(np.linspace(-1, 1, W), np.linspace(-1, 1, H))
        g = np.cos(ang) * gx + np.sin(ang) * gy
        f = f * (1 + rng.uniform(0.2, 0.7) * g[..., None]) + rng.uniform(0, 0.15) * np.clip(g, 0, 1)[..., None]
    f += np.random.default_rng(rng.randint(0, 10**9)).normal(0, rng.uniform(0, 0.03), f.shape)
    return (np.clip(f, 0, 1) * 255).astype(np.uint8)


def composite(src, pool, rng):
    img, rows = read(src)
    row = max(rows, key=lambda r: r[3] * r[4])
    s = rng.uniform(0.18, 0.85) * H
    r = s / max(img.shape[:2])
    small = cv2.resize(img, (max(2, int(img.shape[1] * r)), max(2, int(img.shape[0] * r))))
    h, w = small.shape[:2]
    x0, y0 = rng.randint(0, max(0, W - w)), rng.randint(0, max(0, H - h))
    bg = background(rng, pool)
    mask = np.full((h, w), 255, np.uint8)
    mask = cv2.GaussianBlur(mask, (0, 0), 1.0)                      # soft edge of the crop
    roi = bg[y0:y0 + h, x0:x0 + w].astype(np.float32)
    a = (mask.astype(np.float32) / 255)[..., None]
    bg[y0:y0 + h, x0:x0 + w] = (small * a + roi * (1 - a)).astype(np.uint8)
    out = lighting(bg, rng)
    if rng.random() < 0.5:                                          # mirror (selfie view)
        out = out[:, ::-1].copy(); mirror = True
    else:
        mirror = False
    kp = np.array(row[5:68]).reshape(21, 3)
    px = np.c_[x0 + kp[:, 0] * w, y0 + kp[:, 1] * h]
    if mirror:
        px[:, 0] = W - px[:, 0]
    bw, bh = row[3] * w, row[4] * h
    cx, cy = (x0 + row[1] * w), (y0 + row[2] * h)
    if mirror:
        cx = W - cx
    label = [0, cx / W, cy / H, bw / W, bh / H]
    for (x, y), v in zip(px, kp[:, 2]):
        label += [x / W, y / H, v]
    return out, label


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-train", type=int, default=9000)
    ap.add_argument("--n-val", type=int, default=1500)
    ap.add_argument("--orig", type=int, default=4000, help="original tight crops mixed into train")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    rng = random.Random(a.seed)
    train = [l.strip() for l in open(ROOT / "data" / "train.txt") if l.strip()]
    val = [l.strip() for l in open(ROOT / "data" / "val_sel.txt") if l.strip()]
    def tight(path):   # many dataset images already are webcam-like scenes with a small hand: only crop-like ones get composited
        row = [float(v) for v in pathlib.Path(path.replace("/images/", "/labels/").rsplit(".", 1)[0] + ".txt").read_text().split()[:5]]
        return row[3] * row[4] > 0.25

    for split, src, n in (("train", train, a.n_train), ("val", val, a.n_val)):
        crops = [p for p in src if tight(p)]
        print(split, len(crops), "of", len(src), "are tight crops")
        d = ROOT / "data" / "webcam" / split
        (d / "images").mkdir(parents=True, exist_ok=True); (d / "labels").mkdir(exist_ok=True)
        files = []
        for i in range(n):
            img, label = composite(rng.choice(crops), crops, rng)
            f = d / "images" / f"{i:05d}.jpg"
            cv2.imwrite(str(f), img, [cv2.IMWRITE_JPEG_QUALITY, 90])
            (d / "labels" / f"{i:05d}.txt").write_text(" ".join(f"{v:.6f}" if k else "0" for k, v in enumerate(label)) + "\n")
            files.append(str(f))
        if split == "train":
            files += rng.sample(src, min(a.orig, len(src)))
        (ROOT / "data" / f"{split}_mix.txt").write_text("\n".join(files) + "\n")
    (ROOT / "data" / "hand-webcam.yaml").write_text(f"""path: {ROOT / 'data'}
train: {ROOT / 'data' / 'train_mix.txt'}
val: {ROOT / 'data' / 'val_mix.txt'}
kpt_shape: [21, 3]
flip_idx: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
names:
  0: hand
""")
    print("done")


if __name__ == "__main__":
    main()

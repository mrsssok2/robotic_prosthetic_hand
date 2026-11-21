"""Build docs/results/README.md + figures from tracker_eval.json, latency.json, grasp.json (+ training curves).

    tracking/run_in_docker.sh python tracking/make_report.py
"""
import json
import pathlib
import shutil

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
R = ROOT / "docs" / "results"
COL = {"yolo": "#2a6fdb", "mediapipe": "#6b7280", "classic": "#d9822b"}
FINGERS = ["little", "index", "middle", "ring", "thumb"]


def style(ax):
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.grid(axis="y", alpha=0.25)


def fmt(v, p=3, pct=False):
    if v is None:
        return "n/a"
    return f"{100 * v:.1f}%" if pct else f"{v:.{p}f}"


ev = json.loads((R / "tracker_eval.json").read_text())
acc, speed = ev["accuracy"], ev["speed"]
lat = json.loads((R / "latency.json").read_text()) if (R / "latency.json").exists() else None
grasp = json.loads((R / "grasp.json").read_text()) if (R / "grasp.json").exists() else None

# figures
fig, axes = plt.subplots(1, 2, figsize=(10, 3.6))
w = 0.27
for k, name in enumerate(["yolo", "mediapipe", "classic"]):
    m = acc[name]["curl_mae_per_finger"]
    axes[0].bar(np.arange(5) + (k - 1) * w, [m[f] for f in FINGERS], w, color=COL[name], label=name)
    o = acc[name]["openclosed_acc_per_finger"]
    axes[1].bar(np.arange(5) + (k - 1) * w, [100 * o[f] for f in FINGERS], w, color=COL[name], label=name)
axes[0].set_title("Curl error per finger (MAE, lower is better)", fontsize=10)
axes[1].set_title("Open/closed accuracy per finger (%)", fontsize=10)
axes[1].set_ylim(0, 100)
for ax in axes:
    ax.set_xticks(range(5)); ax.set_xticklabels(FINGERS); style(ax)
axes[0].legend(frameon=False, fontsize=8)
fig.tight_layout(); fig.savefig(R / "curl_per_finger.png", dpi=140); plt.close(fig)

fig, ax = plt.subplots(figsize=(6.5, 3.2))
names = list(speed)
ax.barh(names, [speed[n]["fps"] for n in names], color=[COL[n.split()[0]] for n in names])
for i, n in enumerate(names):
    ax.text(speed[n]["fps"] + 3, i, f'{speed[n]["fps"]:.0f} fps  ({speed[n]["lat_p50_ms"]:.0f} ms p50)', va="center", fontsize=8)
ax.set_xlim(0, max(speed[n]["fps"] for n in names) * 1.35); ax.invert_yaxis(); style(ax)
ax.set_title("Tracker speed, 640x480 frames (this PC, 12-thread CPU / RTX 4050)", fontsize=9)
fig.tight_layout(); fig.savefig(R / "speed.png", dpi=140); plt.close(fig)

run = ROOT / "runs" / "n320" / "results.png"
if run.exists():
    shutil.copy(run, R / "training_curves.png")

# markdown
L = []
L.append("# Results\n")
L.append(f"Own YOLO11 hand-pose tracker vs the MediaPipe baseline vs a classic skin/contour tracker. "
         f"Accuracy on **{ev['test_images']} held-out images** of the Ultralytics hand-keypoints dataset "
         f"(never used for training or checkpoint selection); model `{ev['onnx']}`.\n")
L.append("## Read this first: the labels come from MediaPipe\n")
L.append("The Ultralytics hand-keypoints dataset was auto-labelled by running MediaPipe Hands on the images. "
         "So every \"error\" below is error **against MediaPipe's output**, not against human annotation or true joint angles. "
         "MediaPipe's own row is ~0 by construction (it reproduces its own labels), which says nothing about its real accuracy. "
         "The YOLO model can at best imitate MediaPipe; the numbers measure how well it does. "
         "The curl reference is computed from the label keypoints with the same 2-D curl code (no depth), for every tracker.\n")
L.append("## Accuracy vs the (MediaPipe-derived) labels\n")
L.append("| tracker | hand found | keypoint NME (% of box diag) | PCK@0.05 | curl MAE (0-1) | open/closed acc. |\n|---|---|---|---|---|---|")
for n in ("yolo", "mediapipe", "classic"):
    a = acc[n]
    L.append(f"| {n} | {fmt(a['detection_rate'], pct=True)} | {fmt(a['nme_pct'], 2)} | {fmt(a['pck05'], pct=True)} | {fmt(a['curl_mae'])} | {fmt(a['openclosed_acc'], pct=True)} |")
maj = ev.get("majority_baseline")
if maj:
    L.append(f"| *always-open / majority guess* | - | - | - | - | {fmt(maj, pct=True)} |")
L.append("\nA missed detection counts as wrong for open/closed accuracy. Closed = curl >= 0.5. The classic tracker has no keypoints, "
         "so no keypoint error. Classic thresholds were tuned on the validation split, not the test images.\n")
L.append("![per finger](curl_per_finger.png)\n")
L.append("Per-finger curl MAE:\n\n| tracker | " + " | ".join(FINGERS) + " |\n|---|" + "---|" * 5)
for n in ("yolo", "mediapipe", "classic"):
    m = acc[n]["curl_mae_per_finger"]
    L.append(f"| {n} | " + " | ".join(f"{m[f]:.3f}" for f in FINGERS) + " |")
L.append("\n## Speed and latency\n")
L.append("`tracker.process()` on 640x480 frames (hand crops pasted on a grey canvas; 300 frames after warm-up), Linux container, "
         "same machine for all. Not measured on Windows with a real webcam.\n")
L.append("| tracker | FPS | latency mean | p50 | p95 |\n|---|---|---|---|---|")
for n, s in speed.items():
    L.append(f"| {n} | {s['fps']:.1f} | {s['lat_mean_ms']:.1f} ms | {s['lat_p50_ms']:.1f} ms | {s['lat_p95_ms']:.1f} ms |")
L.append("\n![speed](speed.png)\n")
if lat:
    L.append("### Latency to the Gazebo hand\n")
    L.append(f"One 'all fingers closed' UDP packet, {lat['trials']} trials, wall clock (`scripts/measure_latency.py`). "
             "Includes the controller's smoothing (EMA 0.3 + rate limit 4 curl/s), which is deliberate lag. "
             "Excludes camera capture and the tracker.\n")
    L.append("| stage | median | p95 |\n|---|---|---|")
    for k, label in (("bridge", "UDP -> /hand/state"), ("command", "-> first joint command change"),
                     ("p50", "-> index MCP at 50% in Gazebo"), ("p90", "-> index MCP at 90% in Gazebo")):
        L.append(f"| {label} | {lat[k]['median_ms']:.0f} ms | {lat[k]['p95_ms']:.0f} ms |")
    L.append("\nEnd to end (tracker p50 + time to 50% closure), excluding camera capture:\n")
    L.append("| tracker | tracker p50 | + to 50% closure | = |\n|---|---|---|---|")
    for n, s in speed.items():
        tot = s["lat_p50_ms"] + lat["p50"]["median_ms"]
        L.append(f"| {n} | {s['lat_p50_ms']:.0f} ms | {lat['p50']['median_ms']:.0f} ms | {tot:.0f} ms |")
    L.append("")
if grasp:
    L.append("## Grasp test (Gazebo, scripted)\n")
    L.append("Hand reaches over a table palm-down; fingertip contact sensors + joint effort limit (0.10 N m) bound the closing force; "
             "success = object rises >= 7 cm with the hand and stays within 8 cm of its grasp pose relative to the palm, 1.5 s after a 10 cm lift. "
             "Object start position is jittered by +-6 mm / +-3 mm.\n")
    L.append("| object | grip | success | settled | released on open | peak fingertip force (N, max over trials) |\n|---|---|---|---|---|---|")
    keys = []
    for r in grasp:
        if (r["object"], r["grip"]) not in keys:
            keys.append((r["object"], r["grip"]))
    for o, g in keys:
        rs = [r for r in grasp if r["object"] == o and r["grip"] == g]
        pk = np.max([r["peak_force_N"] for r in rs])
        L.append(f"| {o} | {g} | {sum(r['success'] for r in rs)}/{len(rs)} | {sum(r['settled'] for r in rs)}/{len(rs)} | {sum(r['released'] for r in rs)}/{len(rs)} | {pk:.1f} |")
    L.append("")
L.append("## Limits and caveats\n")
L += [
 "- **No webcam clips.** The build machine (WSL2) has no camera and no recorded hand clips existed, so accuracy is on dataset images, "
 "and speed on synthetic 640x480 frames. `tracking/record_clip.py` (Windows) records clips and `tracking/evaluate.py --clips DIR` scores all "
 "three trackers against MediaPipe on them; that has not been run.",
 "- **YOLO was trained for 23 of the planned 60 epochs** (stopped for time; mAP50-95 pose ~0.81 on the validation split, still rising). "
 "Dataset images are tight hand crops; a hand that is small in a webcam frame is a domain gap that is not measured here.",
 "- **Classic tracker is below the majority-class guess** on open/closed (it is a deliberately simple baseline; thresholds hand-tuned on a few hundred validation images). "
 "It assumes a plain background and a visible forearm.",
 "- **Curl uses 2-D keypoints** for all trackers here (foreshortening under-reads curl); the live MediaPipe path in `hand_cam.py` uses metric world landmarks.",
 "- **Grasping:** only a pinch grip lifts objects. Ball: 0/5 with pinch and with power - the ball gets pushed off the table. The power grip never held anything: "
 "the 3-segment finger with fixed tendon ratios cannot cage an object against a flat palm. Object positions were found by scanning, so success rates are for tuned spots, 5 trials each. "
 "Fingertip forces include contact impulses (peaks up to 8 N); the steady limit is the 0.10 N m joint effort cap (~2.5 N at the tip). "
 "The cup collision is a solid cylinder (visual is an open cup).",
]
(R / "README.md").write_text("\n".join(L) + "\n")
print("wrote", R / "README.md")

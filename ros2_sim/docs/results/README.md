# Results

Everything here is **simulation only** (Gazebo Harmonic, headless, in Docker on WSL2) and measured on one machine. Raw data: the JSON files in this folder; `scripts/summarize_results.py` prints the tables below from them.

**Live tracker: MediaPipe** (default in `hand_cam.py`). The own YOLO11 tracker is kept as a documented experiment (last section); it did not beat MediaPipe.

## 1. Smoothing: EMA -> One Euro filter

The controller used an EMA (alpha 0.3) plus a 4 curl/s rate limit. It is replaced by a One Euro filter per finger (and per wrist angle), run when a camera sample arrives, followed by a 10 curl/s rate limit. Parameters (`min_cutoff` 0.5 Hz, `beta` 2.5) were picked from an offline sweep (`scripts/tune_one_euro.py`, full grid in `one_euro_tuning.txt`) over the same synthetic signals the live measurement uses. `smoothing:=ema` brings the old behaviour back.

Measured against the running simulation (`scripts/measure_latency.py`, `scripts/measure_jitter.py`): a fake camera sends UDP packets at 30 Hz.
*Latency* = one "all fingers closed" packet -> index MCP joint in Gazebo reaches 50% / 90% of its target (wall clock, 15 / 20 trials).
*Jitter* = std of the 50 Hz joint command (index MCP, in curl units) while the finger is held still but the input has Gaussian noise of 0.03 per frame. *Lag* = delay with the best fit between the command and the noise-free sine (finger moving at 0.5 / 1 / 2 Hz, amplitude 0.35, same noise).

| metric | before (EMA) | after (One Euro) | change |
|---|---|---|---|
| UDP -> /hand/state (median) | 0.5 ms | 1.2 ms | +148% |
| UDP -> first joint command (median) | 11 ms | 14 ms | +28% |
| **UDP -> joint at 50% (median)** | 219 ms | 139 ms | -36% |
| UDP -> joint at 50% (p95) | 230 ms | 147 ms | -36% |
| UDP -> joint at 90% (median) | 409 ms | 298 ms | -27% |
| **jitter: std of the command, still hand + noise 0.03 (curl)** | 0.0154 | 0.0111 | -28% |
| jitter: peak-to-peak (curl) | 0.106 | 0.070 | -35% |
| tracking lag, 0.5Hz finger motion | 60 ms | 55 ms | -8% |
| tracking lag, 1.0Hz finger motion | 65 ms | 55 ms | -15% |
| tracking lag, 2.0Hz finger motion | 55 ms | 50 ms | -9% |
| tracking amplitude kept, 0.5Hz | 0.99 | 1.02 | +3% |
| tracking amplitude kept, 1.0Hz | 0.94 | 1.01 | +8% |
| tracking amplitude kept, 2.0Hz | 0.80 | 0.95 | +18% |

Reading it: the jitter drop (-28%) and the step response (-80 ms to 50%, mostly because the rate cap went from 4 to 10 curl/s and the filter lets fast moves through) are real; the lag on *continuous* motion only improved by 5-10 ms, because both filters are about as laggy there (50-65 ms). The noise is synthetic Gaussian; real MediaPipe jitter is not white, so the absolute jitter numbers will differ on a live hand. Camera capture and tracker time are not included: end to end (tracker median + time to 50% closure) is now about 13 + 139 = **152 ms** with MediaPipe on CPU (was 13 + 219 = 232 ms).

## 2. Wrist: 2 degrees of freedom

Two joints between a fixed forearm and the palm: pronation/supination (+-1.4 rad) and flexion/extension (+-1.0 rad). They are driven by the hand's orientation measured from MediaPipe *world* landmarks (palm normal from the index/little knuckles, wrist -> middle-knuckle direction), relative to a neutral pose (forearm vertical, palm to the camera). The sign conventions and the mirroring (selfie view, left hand -> right sim hand) are unit-tested on synthetic hands with known angles (`test_wrist.py`) and were checked on a real clip: with the palm facing the camera and the fingers up the angles read -12 and -18 deg pronation, +6 and +24 flexion (a hand held roughly, not exactly, neutral); over the 45 s clip they ranged -39..+73 deg pronation and -33..+50 deg flexion (`test1.mp4`, MediaPipe, mirrored). There is **no ground truth** for the wrist angle (no motion capture): the numbers show the signs and ranges are plausible, not that they are accurate. The forearm is not visible, so what is measured is the hand's orientation, assumed to be relative to a vertical forearm.

Geometry: the wrist pivots at the centre of the forearm's top face, 35 mm below the palm origin, under a ball-joint dome. Without that gap the palm's edges cut up to 9 mm into the forearm at the joint limits; with it the clearance stays >= 10 mm over a grid of 5 x 5 wrist poses, open hand and fist (`test_geometry.py`, 75 tests incl. the old thumb no-clipping tests). The first design (pivot at the palm base) failed that test, so the limits and the gap come from it.

## 3. Calibration

`hand_cam.py`: `o` = open hand, `f` = fist, `r` = reset, saved to `calibration.json`. Each finger is rescaled so the user's open hand is 0 and their fist 1; the open pose also zeroes the wrist. Rejected when the hand moves during the 1 s capture or a finger changes by < 0.25 between the two poses. Unit-tested (`test_calibration.py`); the keys themselves need a camera and have **not been tried on a live webcam** here.

## 4. Grasping (Gazebo grasp world, scripted, 5 trials per object)

The test closes a simulated hand with a `/hand/state` "Fist" message (nobody names a grip). The controller picks the grip from the hand pose and the nearest object (`select_grip`: power for the ball and cup, pinch for the pen), closes until each finger touches (fingertip force >= 2 N, or the finger is *blocked*: its joint lags its command, which also catches contacts the tip sensor misses), verifies power grasps (>= 2 fingers touching, else re-open and retry up to 3x), then the hand is lifted 10 cm and held for 1.5 s. Success = object up >= 7 cm with the hand and within 8 cm of its grasp point; "released" = it drops when the hand opens. Object start spots are jittered by +-6 mm / +-3 mm; object poses come from the simulator.

| object | before (pinch only, old hand) | auto-grip only (new hand, no stall/verify) | **final** | released on open | peak tip force (N) |
|---|---|---|---|---|---|
| pen | 3/5 | 2/5 | **5/5** pinch | 5/5 | 7.4 |
| cup | 4/5 | 3/5 | **5/5** power | 5/5 | 7.2 |
| ball | 0/5 (pinch and power) | 0/5 | **1/5** power | 5/5 | 7.0 |

Raw: `grasp_before.json`, `grasp_auto_only.json`, `grasp.json`. Read this with care:
- **5 trials per cell is a tiny sample** and the same cell moved between runs (the cup went 3/5 -> 5/5 and, in another run with the optional re-grip switched on, 2/5). Treat 5/5 as "works most of the time here", not as a rate. The "before" column was measured on the earlier build (palm-attached forearm) and is a different, single run.
- What helped: counting a *blocked* finger as touching (the tip sensor only sees the last segment; most holds are by the proximal/middle segments) took the cup from 3/5 to 5/5. Auto grip selection itself did not change the numbers (it picks the grip the old test hard-coded, pinch for the pen); it is what lets one "fist" command work on all three objects. The pen's 5/5 also uses a spot (0.04, -0.085 m) that is the same as before, i.e. a scanned spot, not a general approach.
- **The ball is not solved (1/5).** In ~60 extra trials (spots over the table, wrist flexion 0-1 rad, a pronated side grasp, lower palm, three other finger orders, a keep-closing re-grip) it was never held reliably. Observed cause: with the palm-down hand a ball resting on the table is pushed back by the closing fingers, rolls under the palm and off the table (the forearm and wrist have no collision shapes to stop it); the fingers cannot get under it because of the table. A fix would need a different approach (e.g. wall/back-stop, side grasp with the arm lowered below the current lift range) and was not found in the time spent.
- The verification/retry did not help the cases it was built for: for the pen the retries knocked the pen over, so pinch grasps are accepted with one touching finger (`min_contacts=1`); for the ball a "secure" grasp was often reported (3-4 fingers touching the ball *or the table*) while the ball was not actually held.
- Contacts are counted from joint lag, so a finger pressing on the table counts too.



## 5. Experiment: own YOLO11 hand tracker vs MediaPipe vs classic CV (not the default)

Own YOLO11 hand-pose tracker vs the MediaPipe baseline vs a classic skin/contour tracker. Accuracy on **3000 held-out images** of the Ultralytics hand-keypoints dataset (never used for training or checkpoint selection); model `hand_pose_n640.onnx`.

## Read this first: the labels come from MediaPipe

The Ultralytics hand-keypoints dataset was auto-labelled by running MediaPipe Hands on the images. So every "error" below is error **against MediaPipe's output**, not against human annotation or true joint angles. MediaPipe's own row is ~0 by construction (it reproduces its own labels), which says nothing about its real accuracy. The YOLO model can at best imitate MediaPipe; the numbers measure how well it does. The curl reference is computed from the label keypoints with the same 2-D curl code (no depth), for every tracker.

## Accuracy vs the (MediaPipe-derived) labels

| tracker | hand found | keypoint NME (% of box diag) | PCK@0.05 | curl MAE (0-1) | open/closed acc. |
|---|---|---|---|---|---|
| yolo | 99.9% | 5.92 | 77.9% | 0.081 | 95.3% |
| mediapipe | 100.0% | 0.66 | 98.7% | 0.030 | 99.5% |
| classic | 100.0% | n/a | n/a | 0.437 | 61.8% |
| *always-open / majority guess* | - | - | - | - | 76.0% |

A missed detection counts as wrong for open/closed accuracy. Closed = curl >= 0.5. The classic tracker has no keypoints, so no keypoint error. Classic thresholds were tuned on the validation split, not the test images.

![per finger](curl_per_finger.png)

Per-finger curl MAE:

| tracker | little | index | middle | ring | thumb |
|---|---|---|---|---|---|
| yolo | 0.101 | 0.080 | 0.069 | 0.081 | 0.076 |
| mediapipe | 0.043 | 0.033 | 0.033 | 0.035 | 0.005 |
| classic | 0.468 | 0.417 | 0.428 | 0.447 | 0.425 |

## Speed and latency

`tracker.process()` on 640x480 frames (hand crops pasted on a grey canvas; 300 frames after warm-up), Linux container, same machine for all. Not measured on Windows with a real webcam.

| tracker | FPS | latency mean | p50 | p95 |
|---|---|---|---|---|
| yolo (onnx cpu) | 18.4 | 54.3 ms | 53.1 ms | 63.8 ms |
| mediapipe (cpu) | 65.0 | 15.4 ms | 13.0 ms | 27.1 ms |
| classic (cpu) | 93.1 | 10.7 ms | 9.5 ms | 21.9 ms |
| yolo (onnx cuda) | 104.9 | 9.5 ms | 8.9 ms | 15.4 ms |

![speed](speed.png)

Latency to the Gazebo hand and the end-to-end numbers are in section 1 (the old EMA figures there were superseded). Tracker latency + 139 ms to 50% closure: yolo (onnx cpu) 53 + 139 = 192 ms, mediapipe (cpu) 13 + 139 = 152 ms, classic 10 + 139 = 149 ms, yolo (onnx cuda) 9 + 139 = 148 ms (no camera capture).

## Limits and caveats

- **No webcam clips.** The build machine (WSL2) has no camera and no recorded hand clips existed, so accuracy is on dataset images, and speed on synthetic 640x480 frames. `tracking/record_clip.py` (Windows) records clips and `tracking/evaluate.py --clips DIR` scores all three trackers against MediaPipe on them; that has not been run.
- **YOLO was trained for 23 of the planned 60 epochs** (stopped for time; mAP50-95 pose ~0.81 on the validation split, still rising). Dataset images are tight hand crops; a hand that is small in a webcam frame is a domain gap that is not measured here.
- **Classic tracker is below the majority-class guess** on open/closed (it is a deliberately simple baseline; thresholds hand-tuned on a few hundred validation images). It assumes a plain background and a visible forearm.
- **Curl uses 2-D keypoints** for all trackers here (foreshortening under-reads curl); the live MediaPipe path in `hand_cam.py` uses metric world landmarks.
- **Grasping (old, superseded by section 4):** only a pinch grip lifts objects. Ball: 0/5 with pinch and with power - the ball gets pushed off the table. The power grip never held anything: the 3-segment finger with fixed tendon ratios cannot cage an object against a flat palm. Object positions were found by scanning, so success rates are for tuned spots, 5 trials each. Fingertip forces include contact impulses (peaks up to 8 N); the steady limit is the 0.10 N m joint effort cap (~2.5 N at the tip). The cup collision is a solid cylinder (visual is an open cup).

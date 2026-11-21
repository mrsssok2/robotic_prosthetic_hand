# Self-collected distillation (own webcam clips, MediaPipe as teacher)

Goal: the dataset-trained YOLO (`hand_pose_n640`) lost the hand on a real webcam clip. We auto-labelled our own clips with
MediaPipe and fine-tuned on them mixed with the original data. Model: `models/hand_pose_n640_self.onnx`.

## Real-hand test (`test1.mp4`, 45 s, 1280x720, never used for training)

`tracking/compare_live.py --source test1.mp4` (every 3rd frame = 456 frames; reference = MediaPipe video mode on the same frames).

| | MediaPipe found | YOLO found | keypoint error (% hand-box diag) | curl MAE | open/closed agreement |
|---|---|---|---|---|---|
| before (`hand_pose_n640`) | 93% | 72% | 11.4 | 0.274 | 68.1% |
| **after (`hand_pose_n640_self`)** | 93% | **100%** | **5.6** | **0.185** | **77.5%** |

(Before was re-run here: 68.1% open/closed vs 68.4% in the first run, the rest identical.)
Keypoint error, curl MAE and agreement are computed only on frames where both trackers found a hand.

## No regression on the original data (3000 held-out dataset images, `tracking/evaluate.py`, CPU)

| | found | NME % | curl MAE | open/closed |
|---|---|---|---|---|
| before | 99.9% | 5.92 | 0.0812 | 95.3% |
| after | 99.7% | 6.18 | 0.0811 | 95.2% |

Full after-run: `self_distill/tracker_eval.json`. CPU speed unchanged in kind (15.9 FPS CPU, 111 FPS CUDA in this run; the earlier
run measured 18.4 / 105, same model size; host load differs).

## How it was made

1. `tracking/make_selfcollected_dataset.py` labels `train1.mp4` + `train2.mp4` with MediaPipe in video-tracking mode
   (the same mode as the evaluation reference; static-image mode found the hand in only 16% of candidate frames).
   Kept: handedness score >= 0.85, all 21 keypoints inside the frame, hand diagonal >= 60 px; near-duplicates dropped
   (keypoints moved < 4% of the box diagonal and < 1 s since the last kept frame). Every 2nd frame is a candidate.
   `clip1.mp4` is not a valid recording (`moov atom not found`, 0 frames) and was skipped.
2. Result: **271 train + 38 val frames** (train1: 167 + 38, train2: 104 + 0, the last 20% of each clip is val with a 2 s gap).
   Most rejected frames: hand partly outside the frame (205) and near-duplicates (203); MediaPipe itself found a hand in
   only ~64% of train1 / ~55% of train2 candidates. Labels were checked visually on a 12-frame montage (not committed: the frames contain the user's face).
3. Train list = self frames x3 + 4000 samples of the original mix (composites + dataset images): 4813 images, ~17% self data.
4. Fine-tune from `runs/n640` best weights: 640 px, 25 epochs, lr0 0.002, batch 16, GPU, ~35 min
   (`tracking/train_yolo.py --data data/hand-self.yaml --name n640_self ...`). First attempt (lr0 0.003, patience 10)
   early-stopped at epoch 11 with the epoch-1 weights as "best" (noisy validation, loss still recovering), so it was effectively
   not fine-tuned; discarded and re-run with patience 100.

## Caveats (read these)

- **Teacher = MediaPipe**, so all numbers still mean "agreement with MediaPipe", not accuracy against true joint angles.
- test1 is the same person, room, camera and lighting as the training clips (different recording). The gain is therefore
  an in-domain gain and probably overstates what a new room / person / camera would give. Not tested elsewhere.
- "YOLO found 100%" counts every frame with a detection above 0.25 confidence, including frames where MediaPipe found nothing
  (7%); precision (false hands when no hand is present) is not measured, and there are no hand-free frames in the test.
- The self validation split is only 38 frames, so checkpoint selection was mostly driven by the original-data part of the val list.
- Single run, single seed, one test clip; no confidence intervals.
- 271 training frames from two clips of the same hand motions: expect poses that do not occur in the clips (other grips) to be unchanged.

## Windows GPU (DirectML)

`--tracker yolo --device auto|cuda|dml|cpu` (`hand_cam.py`, `YoloTracker(device=...)`): auto = CUDA, then DirectML, then CPU.
If the accelerated session fails to build or fails its warm-up run, the tracker prints a warning and uses the CPU.
`windows/requirements.txt` now installs `onnxruntime-directml==1.20.1`. **Not tested on Windows or with a real DirectML
device** (no Windows run was possible here): the provider choice and the fallback are unit-tested with a fake onnxruntime
(`src/hand_tracker/test/test_providers.py`, 6 tests); the speed gain on your GPU is unmeasured. Run `python hand_cam.py --tracker yolo`:
it prints `yolo runs on: <provider>`.

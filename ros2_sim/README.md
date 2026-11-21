# ros2_sim (ROS 2 + Gazebo version)

ROS 2 Jazzy, simulation-only version of a robotic prosthetic hand: camera -> MediaPipe -> proportional
finger curl + gestures -> joint commands -> simulated hand (RViz / Gazebo Harmonic).
Original single-file version: https://github.com/mrsssok2/robotic_prosthetic_hand

![demo](docs/img/ros2_sim_demo.gif)

Status: steps 1-6 of 8 (see [docs/design.md](docs/design.md)); metrics, video-file input and CI come next.

![open hand, thumb half, thumb closed, fist](docs/img/hand_sequence.png)


## Own hand tracker vs MediaPipe vs classic CV, grips and grasping

`hand_cam.py --tracker yolo|mediapipe|classic` swaps the tracker; the rest of the pipeline is unchanged.

![pipeline](docs/img/tracker_pipeline.svg)

- **yolo**: YOLO11n-pose trained on the Ultralytics hand-keypoints dataset (21 keypoints) on the RTX 4050, exported to ONNX
  (`models/hand_pose_n320.onnx`, runs with onnxruntime only). Training: `tracking/train_yolo.py`.
- **classic**: skin segmentation + palm centre + radial profile -> open/closed per finger (`trackers/classic_tracker.py`).
- **Honest caveat:** the dataset labels were produced by MediaPipe, so "error" means "difference from MediaPipe", and MediaPipe scores ~0 against its own labels.

| (3000 held-out images) | hand found | keypoint NME | curl MAE | open/closed acc. | FPS 640x480 | latency p50 |
|---|---|---|---|---|---|---|
| yolo (ONNX, CPU / CUDA) | 99.9% | 6.2% of box diag | 0.075 | 95.3% | 34 / 78 | 28 / 12 ms |
| mediapipe (CPU) | 100% | ~0.7% (reference) | ~0.03 (reference) | ~99.5% | 39 | 22 ms |
| classic (CPU) | 100% | n/a | 0.435 | 61.9% (majority guess: 75.9%) | 57 | 16 ms |

UDP packet to the Gazebo hand reaching 50% closure: 226 ms median (mostly the controller's smoothing); add the tracker latency for end to end (about 240-255 ms, no camera).
Not done: evaluation on real webcam clips (no camera in WSL; `tracking/record_clip.py` + `evaluate.py --clips` are ready), and YOLO was stopped at 23/60 epochs.
Full tables, per-finger numbers and caveats: [docs/results/README.md](docs/results/README.md).

**Grips** (`hand_controller/grips.py`): open, point, pinch, power as coordinated preshape -> close poses; `/hand/grip` topic or `grip_assist:=true` (gestures Open Hand / Pointing / Pinch / Fist). Closing is **force-limited**: each finger stops when its fingertip contact sensor reads >= 2 N and the joint effort limit caps the squeeze.

**Grasp world**: `./run.sh grasp:=true` (table with ball, cup, pen, palm-down hand with a lift joint, contact sensors). Scripted test `scripts/grasp_test.py`, 5 trials each: **pen 3/5, cup 4/5, ball 0/5** (pinch grip). The power grip did not hold anything (finger geometry), so only the pinch grip is demonstrated.

## Run it (webcam -> Gazebo hand)

MediaPipe (Python, like the original `gesture_control.py`) runs on **Windows**, because WSL2/Docker cannot
see the webcam. It sends the five finger values over UDP to the ROS bridge in Docker.

1. WSL: `cd ~/projects/robotic_prosthetic_hand/ros2_sim && ./run.sh` (builds in Docker, opens Gazebo; `./run.sh input:=fake` = no camera)
2. Windows: one-time setup and `python hand_cam.py` -> [windows/README.md](windows/README.md)

Optional alternative: `./run.sh input:=browser` and open http://localhost:8765/ (MediaPipe JS in the browser).
Needs the `gurdy2/ros:jazzy` image and Docker Desktop with WSLg.

```
hand_cam.py -udp 9870-> udp_bridge -/hand/state-> controller_node -+-> /hand/servo_angles (0-180 deg, ch 0-4, thumb reversed)
(Windows)                                                           +-> /finger_position_controller/commands -> Gazebo (16 joints)
```

Test the ROS side without any camera: `scripts/fake_udp.py 0 0 0 0 1` (little index middle ring thumb, 0..1).

## Tests

```bash
python3 -m venv .venv && . .venv/bin/activate && pip install numpy pytest
(cd src/hand_tracker && python -m pytest) && (cd src/hand_controller && python -m pytest)
# hand geometry (needs ROS for xacro):  docker run --rm -v $PWD:/home/ubuntu/hand_ros2 hand_ros2:jazzy bash -c \
#   'source /opt/ros/jazzy/setup.bash; cd ~/hand_ros2/src/hand_description && python3 -m pytest test'
```

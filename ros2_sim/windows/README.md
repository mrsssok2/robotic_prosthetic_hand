# Windows camera script (`hand_cam.py`)

WSL2/Docker cannot see the webcam, so MediaPipe runs on Windows and sends the five finger values
(plus the gesture) over UDP to the ROS bridge in Docker (`localhost:9870`).

## One-time setup (PowerShell)

1. **Python 3.11** (MediaPipe 0.10.14 needs 3.8-3.11): install from https://www.python.org/downloads/release/python-3119/
   ("Windows installer (64-bit)", tick *Add python.exe to PATH*), or `winget install Python.Python.3.11`.
2. **Copy the code to Windows** (the script imports the tested tracking math from `src/hand_tracker`):
   ```powershell
   robocopy \\wsl.localhost\Ubuntu\home\mahadev\projects\robotic_prosthetic_hand\ros2_sim C:\hand_ros2 /E /XD .git build install log .venv
   ```
   (Re-run this after pulling changes. Running straight from the `\\wsl.localhost\...` path also works if you prefer.)
3. **Virtual environment + packages**:
   ```powershell
   cd C:\hand_ros2\windows
   py -3.11 -m venv .venv
   .\.venv\Scripts\Activate.ps1
   pip install -r requirements.txt        # mediapipe 0.10.14, opencv-python 4.10, numpy 1.26
   ```
   If PowerShell blocks the activate script: `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`.

## Every time

1. In **WSL**: `cd ~/projects/robotic_prosthetic_hand/ros2_sim && ./run.sh` (Gazebo opens and waits for finger values).
2. On **Windows** (venv active):
   ```powershell
   cd C:\hand_ros2\windows
   python hand_cam.py
   ```
   A preview window shows the mirrored camera, the landmarks, FPS, the gesture and the five finger
   values (Little, Index, Middle, Ring, Thumb). Press **q** in that window to quit.

## Calibration (30 seconds, optional but recommended)

People's hands read differently (a flat hand may be 0.15 curl, a tight fist 0.8), so the simulated hand may never fully open or
close. In the preview window:

1. Hold your hand **flat and open**, palm to the camera, fingers up, forearm vertical, and press **`o`** (about 1 s, the bar at the bottom counts it).
2. Make a **tight fist** and press **`f`**.

Every finger is then rescaled so *your* open hand reads 0 and *your* fist reads 1; the open pose also sets the wrist's neutral
position. It is saved to `calibration.json` next to `hand_cam.py` and loaded next time (`--calibration FILE`, `--no-calibration`);
**`r`** clears it. If the hand moves during the capture, or a finger barely changes between the two poses, the preview says so.

## Reach (grasp world)

`./run.sh grasp:=true auto_grip:=true` in WSL, then in the preview press **`m`**: your hand's position in the image moves the
simulated hand over the table (left/right, up = further, closer to the camera = lower). **`c`** re-centres (do it with the hand
where it is comfortable), `m` again turns it off. `python hand_cam.py --reach` starts with it on. Then move over the cup, lower the
hand and make a fist; open the hand to release.

## Wrist

With the default MediaPipe tracker the hand's **pronation/supination** and **flexion/extension** are measured from the world
landmarks (palm normal and wrist -> middle-knuckle direction) and drive the two wrist joints of the simulated hand. They are the
orientation of the hand relative to the calibrated neutral pose (forearm vertical in the image, elbow down, palm to the camera,
fingers up), so rotate your **hand**, not your whole arm. The preview shows `wrist pron .. flex .. deg`. (`--tracker yolo|classic`
have no depth, so the wrist stays neutral.)

Options: `--camera 1` (other webcam), `--source clip.mp4` (video file or still image; add `--mirror-source` if it is a selfie recording made by `tracking/record_clip.py`), `--host/--port`
(default `127.0.0.1:9870`), `--no-window`.

## Thumb

Like the original (which looked at the thumb's x), the thumb value is how far the tip has moved
sideways across the palm, measured along the palm's own index-to-little-finger axis, so it does not
depend on how the hand is rotated. 0 = thumb stuck out to the side, 1 = tip across the palm.

## Troubleshooting

- *Cannot open the camera*: close other apps using it, try `--camera 1`.
- *Nothing moves in Gazebo*: the preview should show a hand; check Docker Desktop is running and that
  `./run.sh` shows `listening on udp/9870`. Test the ROS side without a camera: `scripts/fake_udp.py`.

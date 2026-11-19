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

Options: `--camera 1` (other webcam), `--source clip.mp4` (video file or still image), `--host/--port`
(default `127.0.0.1:9870`), `--no-window`.

## Thumb

Like the original (which looked at the thumb's x), the thumb value is how far the tip has moved
sideways across the palm, measured along the palm's own index-to-little-finger axis, so it does not
depend on how the hand is rotated. 0 = thumb stuck out to the side, 1 = tip across the palm.

## Troubleshooting

- *Cannot open the camera*: close other apps using it, try `--camera 1`.
- *Nothing moves in Gazebo*: the preview should show a hand; check Docker Desktop is running and that
  `./run.sh` shows `listening on udp/9870`. Test the ROS side without a camera: `scripts/fake_udp.py`.

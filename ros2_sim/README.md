# ros2_sim (ROS 2 + Gazebo version)

ROS 2 Jazzy, simulation-only version of a robotic prosthetic hand: camera -> MediaPipe -> proportional
finger curl + gestures -> joint commands -> simulated hand (RViz / Gazebo Harmonic).
Original single-file version: https://github.com/mrsssok2/robotic_prosthetic_hand

<!-- TODO: replace with a demo GIF -->
![demo](docs/demo.gif)

Status: steps 1-6 of 8 (see [docs/design.md](docs/design.md)); metrics, video-file input and CI come next.

![open hand, thumb half, thumb closed, fist](docs/img/hand_sequence.png)

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

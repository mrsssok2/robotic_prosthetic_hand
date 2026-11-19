# Design

Simulation-only ROS 2 (Jazzy) rebuild of the robotic prosthetic hand
(original: single-file `gesture_control.py`, MediaPipe -> servo angles on PCA9685).

## Dataflow

```
 Windows hand_cam.py (MediaPipe Python) --UDP--> udp_bridge ──────────────┐ (HandState, skips tracker)
 browser (MediaPipe JS, optional) --WebSocket--> ws_bridge ─┐
 video file / rosbag ──> hand_tracker (MediaPipe) ┼─> /hand/landmarks ─> hand_tracker core
 (webcam: optional, native Linux only)            ┘                      │
                                                                         v
   /hand/state (curl[5], gesture, handedness) ─> hand_controller ─> /hand/joint_commands
                                                                         │
                       robot_state_publisher + RViz  <── /joint_states ──┤
                       Gazebo Harmonic + ros2_control (position ctrl) <──┘
```

## Packages

| package | build | role |
|---|---|---|
| `hand_interfaces` | ament_cmake | `HandLandmarks.msg`, `HandState.msg` |
| `hand_tracker` | ament_python | `core/` (pure numpy math), tracker node, video/bag input, WS bridge |
| `hand_controller` | ament_python | curl -> joint commands; smoothing, rate + joint limits |
| `hand_description` | ament_cmake | xacro URDF, RViz config, controller YAML |
| `hand_bringup` | ament_python | launch files, Gazebo world, metrics tools |

## Key decisions

- **Finger order = servo channels 0-4:** Little, Index, Middle, Ring, Thumb. Every array in
  the system uses this order.
- **Curl from 3D joint angles**, not tip-vs-PIP y. Angle between consecutive bones is invariant to
  rotation, scale, translation and mirroring, so left/right hands need no special case.
  Curl = clip((sum of joint flexions - open) / (closed - open)); ranges are configurable.
  **Thumb** is measured like the original (by x): sideways position of the tip along the palm's
  index-MCP -> little-MCP axis, in palm widths (rotation/mirror invariant), mapped -0.5..0.6 -> 0..1.
- **ROS-free core.** All math lives in `hand_tracker/core` (numpy only), unit-tested without ROS.
- **Camera in WSL2:** webcam passthrough is painful, so `windows/hand_cam.py` (OpenCV + MediaPipe Python,
  as in the original) runs on Windows, reuses `core/` for curl + gesture and sends 5 values over UDP.
  The browser page + WebSocket bridge stays as an optional alternative.
- **Controller topic interface matches a future real driver:** angle in degrees per channel
  (0/180 range, thumb reversed as in the original). A real PCA9685 node would subscribe to the
  same topic; simulation maps it onto joint positions.
- **Fingers are tendon-coupled:** one command per finger drives MCP/PIP/DIP with fixed ratios (1 : 1.2 : 0.7);
  the thumb sits low on the +X side and points sideways about 70 deg from the index finger when open;
  ONE thumb value drives its swing across the front of the palm (main motion, in the palm plane), a little
  opposition and two curling segments (all scaled together).
- **Look:** InMoov-style white PLA hand. Meshes are procedural (`hand_description/scripts/gen_meshes.py`,
  dimensions in `scripts/dims.py`), visual only; no collision shapes, gravity off in the worlds (the weak position
  controller cannot hold the thumb against gravity). Gazebo Harmonic's DART ignores URDF `<mimic>`,
  so in simulation hand_controller applies the ratios itself and all 15 joints are position-driven;
  the `<mimic>` tags serve RViz. A test keeps both tables equal.
- **Resources:** 8 GB WSL RAM / 6 GB GPU -> MediaPipe on CPU, Gazebo headless-capable, small hand mesh.

## Metrics

Latency (capture timestamp in header -> joint command stamp), tracking accuracy vs. a labelled
recorded clip (curl MAE, gesture accuracy), FPS of each stage.

## Plan

1. **Core tracking math** (done): repo scaffold, per-finger curl from 3D angles, gesture rules, tests.
2. **(done)** ROS interfaces + hand_tracker node:** msgs, Dockerfile, MediaPipe node on video file, topics, bag support.
3. **(done)** Browser bridge: MediaPipe Tasks JS page + WebSocket->ROS node, header stamps for latency.
4. **(done)** hand_controller: curl -> servo-angle commands, EMA smoothing, rate/joint limits, thumb opposition.
5. **(done)** hand_description: xacro 5-finger URDF (tendon coupling, 2-DOF thumb), RViz-only launch.
6. **(done)** Gazebo + ros2_control: Harmonic world, position controllers, gz_ros2_control.
7. **Launch + metrics:** browser->Gazebo, video->sim, RViz-only launches; latency/accuracy/FPS tools.
8. **Polish:** integration tests, CI, README with GIF, tuning.

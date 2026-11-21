"""Wrist pronation/supination and flexion/extension from MediaPipe *world* landmarks (numpy only).

Only the hand is visible, not the forearm, so the wrist angles are the orientation of the hand relative to a **neutral pose
with the forearm vertical in the image (elbow down), palm facing the camera, fingers up** (this is also the pose you
calibrate with 'o' in hand_cam.py, which removes the remaining offset). Two measurements from the landmarks:

    wrist -> middle MCP (0 -> 9)      the finger axis       z_h
    pinky MCP -> index MCP (17 -> 5)  across the palm       x_h  (towards the thumb side)
    palm normal                       y_h = z_h x x_h       (pointing out of the back of the hand)

These three axes are the sim hand's palm frame (fingers +Z, thumb side +X, back of the hand +Y). The rotation of that frame
relative to the neutral pose is split as  R = Rz(roll) @ Rx(flex) @ Ry(deviation)  -- the same joint order as the URDF
(roll about the forearm axis, then flexion about the transverse axis); the ulnar/radial deviation is not modelled, so it
is dropped.

    pronation  > 0 pronation (palm turns away from the thumb side), < 0 supination        [rad]
    flexion    > 0 flexion (fingers tip towards the palm), < 0 extension                   [rad]

World landmarks are in the camera frame (x right, y down, z away from the camera, metres). hand_cam.py mirrors the frame
for the selfie view, so pass mirrored=True for it (the default); a left hand is mirrored into a right hand because the
simulated hand is a right hand. `label` is MediaPipe's handedness label of the (possibly mirrored) input.
"""
from __future__ import annotations

import numpy as np

from .landmarks import INDEX_MCP, LITTLE_MCP, NUM_LANDMARKS

WRIST, MIDDLE_MCP = 0, 9
_EPS = 1e-9

# Joint limits of the simulated wrist (rad), the same numbers as WRIST_* in the URDF (a test keeps them in sync):
# (pronation, flexion) as (lower, upper); pronation + / supination -, flexion + / extension -.
PRONATION_LIMITS = (-1.4, 1.4)
FLEXION_LIMITS = (-1.0, 1.0)

# neutral hand frame in camera coordinates: x_s thumb side, y_s back of the hand, z_s fingers
_NEUTRAL = np.column_stack([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])


def hand_rotation(landmarks, label: str | None = None, mirrored: bool = True) -> np.ndarray:
    """3x3 rotation whose columns are the hand's (thumb-side, back-of-hand, finger) axes in the camera frame.

    The camera-frame axes are first corrected so that the hand is a right hand seen by a normal (un-mirrored) camera."""
    pts = np.asarray(landmarks, float)
    if pts.shape != (NUM_LANDMARKS, 3):
        raise ValueError(f"expected ({NUM_LANDMARKS}, 3) landmarks, got {pts.shape}")
    pts = pts.copy()
    left = label is not None and (label.lower() == "left") == mirrored    # the true hand is a left hand
    if mirrored != left:      # undo the selfie mirror; and/or mirror a left hand into a right one
        pts[:, 0] *= -1.0
    z = pts[MIDDLE_MCP] - pts[WRIST]
    z = z / max(np.linalg.norm(z), _EPS)
    x = pts[INDEX_MCP] - pts[LITTLE_MCP]
    x = x - np.dot(x, z) * z
    x = x / max(np.linalg.norm(x), _EPS)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def wrist_angles(landmarks, label: str | None = None, mirrored: bool = True) -> np.ndarray:
    """(pronation, flexion) in radians relative to the neutral pose (see the module docstring)."""
    rel = _NEUTRAL.T @ hand_rotation(landmarks, label, mirrored)
    flex = np.arcsin(np.clip(rel[2, 1], -1.0, 1.0))
    roll = np.arctan2(-rel[0, 1], rel[1, 1])           # about +Z (towards supination for a right hand)
    return np.array([-roll, flex])

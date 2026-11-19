"""Per-finger proportional curl (0 = straight, 1 = fully closed) from 3D joint angles.

Fingers use joint angles. The thumb is different, as in the original project (which
measured it by x): its value is how far the tip has swept *sideways across the palm*,
measured along the palm's own lateral axis (index MCP -> little MCP), so it is still
independent of hand rotation, scale and mirroring.

Why angles: the angle between consecutive bone vectors is invariant to hand
rotation, translation, uniform scale and mirroring, so the same code handles
left and right hands at any orientation. (The original project compared tip-vs-PIP
image y, which breaks as soon as the hand is rotated.)

Input is a (21, 3) array of MediaPipe landmarks. Prefer *world* landmarks
(metric); normalized image landmarks also work because x/y/z share a scale.
Flexion is unsigned, so slight hyperextension reads as a small positive angle;
`open_deg` absorbs that.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .landmarks import FINGER_CHAINS, FINGER_ORDER, INDEX_MCP, LITTLE_MCP, NUM_LANDMARKS, THUMB_TIP

_EPS = 1e-9


@dataclass(frozen=True)
class CurlConfig:
    """Total flexion (deg, summed over the measured joints) mapped to curl 0..1."""

    # finger: (open_deg, closed_deg)
    ranges: dict = field(default_factory=lambda: {
        "little": (20.0, 200.0),
        "index": (20.0, 200.0),
        "middle": (20.0, 200.0),
        "ring": (20.0, 200.0),
    })
    # thumb sweep s (see thumb_sweep): s <= thumb_open -> 0, s >= thumb_closed -> 1
    thumb_open: float = -0.5
    thumb_closed: float = 0.6


def _as_landmarks(landmarks) -> np.ndarray:
    pts = np.asarray(landmarks, dtype=float)
    if pts.shape != (NUM_LANDMARKS, 3):
        raise ValueError(f"expected ({NUM_LANDMARKS}, 3) landmarks, got {pts.shape}")
    return pts


def _flexion_deg(chain_pts: np.ndarray) -> np.ndarray:
    """Angle (deg) between consecutive bones at each interior joint of a chain."""
    bones = np.diff(chain_pts, axis=0)
    norms = np.linalg.norm(bones, axis=1, keepdims=True)
    units = bones / np.maximum(norms, _EPS)
    cosines = np.clip(np.sum(units[:-1] * units[1:], axis=1), -1.0, 1.0)
    return np.degrees(np.arccos(cosines))


def joint_flexions(landmarks) -> dict[str, np.ndarray]:
    """Per-finger flexion angles in degrees (MCP, PIP, DIP); the thumb is not a flexion chain."""
    pts = _as_landmarks(landmarks)
    return {name: _flexion_deg(pts[list(chain)]) for name, chain in FINGER_CHAINS.items()}


def thumb_sweep(landmarks) -> float:
    """Signed sideways position of the thumb tip across the palm, in palm widths.

    0 is the index MCP, +1 is the little-finger MCP; negative = thumb stuck out to the side.
    """
    pts = _as_landmarks(landmarks)
    lateral = pts[LITTLE_MCP] - pts[INDEX_MCP]
    width = max(np.linalg.norm(lateral), _EPS)
    return float(np.dot(pts[THUMB_TIP] - pts[INDEX_MCP], lateral) / width**2)


def finger_curls(landmarks, config: CurlConfig | None = None) -> np.ndarray:
    """Curl per finger in FINGER_ORDER (little, index, middle, ring, thumb), each in [0, 1]."""
    cfg = config or CurlConfig()
    flex = joint_flexions(landmarks)
    out = np.empty(len(FINGER_ORDER))
    for i, name in enumerate(FINGER_ORDER[:4]):
        lo, hi = cfg.ranges[name]
        out[i] = np.clip((flex[name].sum() - lo) / (hi - lo), 0.0, 1.0)
    out[4] = np.clip((thumb_sweep(landmarks) - cfg.thumb_open) / (cfg.thumb_closed - cfg.thumb_open), 0.0, 1.0)
    return out

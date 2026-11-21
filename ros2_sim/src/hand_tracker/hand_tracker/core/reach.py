"""Reach control: where the hand is in the camera image -> where the simulated hand goes over the table (numpy only).

    palm centre x (image, right +)         ->  x  (hand moves right)
    palm centre y (image, up +)            ->  y  (hand moves away from the base, over the table)
    apparent hand size (wrist -> middle MCP pixel length, bigger = closer to the camera)  ->  closeness (lower the hand)

The mapper outputs three numbers in -1..1 relative to a reference pose (the pose at 'c' re-centre = the middle of the
workspace), with a dead zone so a hand held roughly still does not creep. hand_controller scales them to metres, smooths them with
a One Euro filter and clamps them to the workspace. Size works as a depth cue because the hand is about the same size for
everyone: moving the hand towards the camera by a factor RATIO (default 1.6x bigger) lowers the simulated hand over its whole height range.
"""
from __future__ import annotations

import numpy as np

PALM_POINTS = (0, 5, 9, 13, 17)      # wrist + the four knuckles
WRIST, MIDDLE_MCP = 0, 9


def palm_centre(kp) -> np.ndarray:
    return np.asarray(kp, float)[list(PALM_POINTS), :2].mean(axis=0)


def hand_size(kp) -> float:
    k = np.asarray(kp, float)
    return float(np.linalg.norm(k[MIDDLE_MCP, :2] - k[WRIST, :2]))


def deadzone(x, dz: float):
    """0 inside +-dz, then linear to +-1 at +-1 (no jump at the edge)."""
    x = np.asarray(x, float)
    return np.sign(x) * np.clip((np.abs(x) - dz) / (1.0 - dz), 0.0, 1.0)


class ReachMapper:
    def __init__(self, frame_w, frame_h, range_xy=0.25, ratio=1.6, dead=0.08):
        self.w, self.h = float(frame_w), float(frame_h)
        self.range_xy, self.ratio, self.dead = range_xy, ratio, dead   # range_xy: image fraction (of the height) for full travel
        self.ref = None

    def _measure(self, kp):
        c = palm_centre(kp)
        return np.array([c[0] / self.h, c[1] / self.h, hand_size(kp) / self.h])

    def recentre(self, kp) -> None:
        self.ref = self._measure(kp)

    def map(self, kp) -> np.ndarray:
        """[x, y, closeness] in -1..1 (zeros until the first recentre, which happens on the first call)."""
        if self.ref is None:
            self.recentre(kp)
        m = self._measure(kp)
        raw = np.array([(m[0] - self.ref[0]) / self.range_xy,
                        -(m[1] - self.ref[1]) / self.range_xy,
                        np.log(max(m[2], 1e-6) / max(self.ref[2], 1e-6)) / np.log(self.ratio)])
        return deadzone(np.clip(raw, -1.0, 1.0), self.dead)

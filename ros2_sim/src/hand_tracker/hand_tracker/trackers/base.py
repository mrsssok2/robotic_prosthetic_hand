"""Tracker interface shared by hand_cam.py and the evaluation."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..core import finger_curls


@dataclass
class HandResult:
    curl: np.ndarray                      # (5,) little, index, middle, ring, thumb in 0..1
    keypoints: np.ndarray | None = None   # (21, 2) image pixels (None for the classic tracker)
    score: float = 1.0
    extra: dict | None = None             # e.g. MediaPipe world-landmark curl


class Tracker:
    name = "base"

    def process(self, bgr: np.ndarray) -> HandResult | None:  # pragma: no cover - interface
        raise NotImplementedError

    def close(self) -> None:
        pass


def curl_from_image_keypoints(kpts_xy: np.ndarray) -> np.ndarray:
    """Curl from 21 2-D keypoints (z = 0). Foreshortening when fingers point at the camera
    makes this under-read compared with metric 3-D landmarks; all trackers are scored the same way."""
    pts = np.zeros((21, 3))
    pts[:, :2] = kpts_xy
    return finger_curls(pts)

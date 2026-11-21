"""Quick per-user calibration of the finger curls (numpy + json only).

People's hands differ: a flat hand may read 0.15 and a tight fist 0.8 instead of 0 and 1, so the simulated hand never fully
opens or closes. Two reference poses fix that:

    open   (key 'o' in hand_cam.py)   hold the hand flat and open      -> every finger's curl maps to 0
    fist   (key 'f')                  make a tight fist                 -> every finger's curl maps to 1

Per finger  calibrated = clip((raw - open) / (fist - open), 0, 1).  The open pose also records the raw wrist angles as the
wrist's neutral pose (hand flat, palm to the camera, fingers up), which are subtracted from later wrist angles.
A finger whose open and fist readings are closer than ``MIN_SPAN`` (the pose was not really made, or the tracker lost that
finger) keeps the identity mapping and is reported. Saved as JSON; missing poses default to open = 0, fist = 1.
"""
from __future__ import annotations

import json
import pathlib
import time
from dataclasses import dataclass, field

import numpy as np

MIN_SPAN = 0.25          # smallest accepted (fist - open) curl difference of a finger
MAX_SPREAD = 0.15        # a capture is rejected if any value's std over the capture is larger (the hand was moving)
CAPTURE_FRAMES = 30      # valid frames per capture (~1 s at 30 fps)
CAPTURE_TIMEOUT = 6.0    # s


@dataclass
class Calibration:
    open: np.ndarray = field(default_factory=lambda: np.zeros(5))
    fist: np.ndarray = field(default_factory=lambda: np.ones(5))
    wrist_neutral: np.ndarray = field(default_factory=lambda: np.zeros(2))
    has_open: bool = False
    has_fist: bool = False

    def span_ok(self):
        return (self.fist - self.open) >= MIN_SPAN

    def apply(self, curl) -> np.ndarray:
        """Raw curl (5,) -> calibrated curl in 0..1 (fingers that failed the span check are passed through)."""
        c = np.asarray(curl, float)
        ok = self.span_ok()
        span = np.where(ok, self.fist - self.open, 1.0)
        base = np.where(ok, self.open, 0.0)
        return np.clip((c - base) / span, 0.0, 1.0)

    def apply_wrist(self, wrist) -> np.ndarray:
        return np.asarray(wrist, float) - self.wrist_neutral

    def to_dict(self) -> dict:
        return {"version": 1, "created": time.strftime("%Y-%m-%d %H:%M:%S"),
                "open": [round(float(v), 4) for v in self.open], "fist": [round(float(v), 4) for v in self.fist],
                "wrist_neutral": [round(float(v), 4) for v in self.wrist_neutral],
                "has_open": self.has_open, "has_fist": self.has_fist}

    @classmethod
    def from_dict(cls, d: dict) -> "Calibration":
        cal = cls(np.array(d["open"], float), np.array(d["fist"], float), np.array(d.get("wrist_neutral", [0, 0]), float),
                  bool(d.get("has_open", True)), bool(d.get("has_fist", True)))
        if cal.open.shape != (5,) or cal.fist.shape != (5,) or cal.wrist_neutral.shape != (2,):
            raise ValueError("calibration file has the wrong shape")
        return cal

    def save(self, path) -> None:
        pathlib.Path(path).write_text(json.dumps(self.to_dict(), indent=1))

    @classmethod
    def load(cls, path) -> "Calibration | None":
        """The saved calibration, or None if there is no (valid) file."""
        try:
            return cls.from_dict(json.loads(pathlib.Path(path).read_text()))
        except (OSError, ValueError, KeyError, TypeError):
            return None


class CaptureSession:
    """Collects ``CAPTURE_FRAMES`` valid frames of a reference pose, then takes the median.

    start("open" | "fist", now) ... feed(curl, wrist, now) each frame (curl None = no hand) ... -> status string; when the
    capture finishes, ``self.calibration`` is updated and ``message`` says what happened."""

    def __init__(self, calibration: Calibration | None = None):
        self.calibration = calibration or Calibration()
        self.kind = None
        self.message = ""
        self.ok = False            # the last finished capture was stored
        self._curls, self._wrists, self._t0 = [], [], 0.0

    @property
    def active(self) -> bool:
        return self.kind is not None

    @property
    def progress(self) -> float:
        return len(self._curls) / CAPTURE_FRAMES if self.active else 0.0

    def start(self, kind: str, now: float) -> None:
        if kind not in ("open", "fist"):
            raise ValueError(kind)
        self.kind, self._curls, self._wrists, self._t0 = kind, [], [], now
        self.ok = False
        self.message = f"calibrating {kind}: hold the pose..."

    def reset(self) -> None:
        self.calibration, self.kind = Calibration(), None
        self.message = "calibration cleared"

    def feed(self, curl, wrist, now: float) -> bool:
        """True when a capture just finished (successfully or not)."""
        if not self.active:
            return False
        if curl is not None:
            self._curls.append(np.asarray(curl, float))
            self._wrists.append(np.zeros(2) if wrist is None else np.asarray(wrist, float))
        if len(self._curls) >= CAPTURE_FRAMES:
            self._finish()
            return True
        if now - self._t0 > CAPTURE_TIMEOUT:
            self.kind, self.ok, self.message = None, False, "calibration failed: no hand seen, try again"
            return True
        return False

    def _finish(self) -> None:
        curls, wrists = np.array(self._curls), np.array(self._wrists)
        kind, self.kind = self.kind, None
        if curls.std(axis=0).max() > MAX_SPREAD:
            self.message = f"{kind}: the hand moved, hold still and try again"
            return
        med = np.median(curls, axis=0)
        cal = self.calibration
        if kind == "open":
            cal.open, cal.has_open, cal.wrist_neutral = med, True, np.median(wrists, axis=0)
        else:
            cal.fist, cal.has_fist = med, True
        bad = [i for i, ok in enumerate(cal.span_ok()) if not ok]
        names = ("little", "index", "middle", "ring", "thumb")
        note = f" (not calibrated, too little difference: {', '.join(names[i] for i in bad)})" if bad and cal.has_open and cal.has_fist else ""
        self.ok, self.message = True, f"{kind} saved{note}"

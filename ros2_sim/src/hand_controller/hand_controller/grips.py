"""Grip types as coordinated finger poses, and a force-limited closing planner (ROS-free, numpy only).

A grip is two poses in curl space (little, index, middle, ring, thumb; 0 open .. 1 closed):

    preshape  hand shaped for the approach (fingers half open, thumb opposed ...)
    close     final pose when nothing is in the way

``GripPlanner`` runs  open/any -> preshape -> close -> hold. While closing, every finger advances towards its final pose
at its own pace (the thumb can start late) and **stops as soon as its fingertip force reaches ``f_hold``**; if the force
exceeds ``f_max`` it backs off a little. So a ball, a cup and a pen end up held by whichever fingers touch them, with
a bounded force (the simulated joint effort limit is the hard cap, this is the soft limit).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

FINGERS = ("little", "index", "middle", "ring", "thumb")


@dataclass(frozen=True)
class Grip:
    name: str
    preshape: tuple
    close: tuple
    delay: tuple = (0.0, 0.0, 0.0, 0.0, 0.0)   # seconds before each finger starts closing
    description: str = ""


GRIPS = {g.name: g for g in (
    Grip("open", (0, 0, 0, 0, 0), (0, 0, 0, 0, 0), description="all fingers extended"),
    Grip("point", (0.4, 0, 0.4, 0.4, 0.2), (1, 0, 1, 1, 0.6), (0, 0, 0.05, 0.1, 0.2),
         "index extended, the other fingers and the thumb folded"),
    Grip("pinch", (0, 0.3, 0, 0, 0.3), (0, 0.9, 0, 0, 0.5), (0, 0, 0, 0, 0),
         "index tip against the thumb tip (precision grip, e.g. a pen); the other fingers stay open"),
    Grip("power", (0.3, 0.3, 0.3, 0.3, 0.3), (1, 1, 1, 1, 0.6), (0.2, 0.0, 0.0, 0.1, 1.0),
         "all fingers wrap the object, thumb closes last (ball, cup)"),
)}

# tracker gesture -> grip, for grip-assist mode
GESTURE_TO_GRIP = {"Open Hand": "open", "Pointing": "point", "Pinch": "pinch", "Fist": "power"}


class GripPlanner:
    """phase: idle -> preshape -> closing -> holding;  'release' goes back to the open pose."""

    def __init__(self, f_hold=2.0, f_max=5.0, preshape_time=0.6, close_time=1.6, settle_time=0.3, backoff=0.25):
        self.f_hold, self.f_max = f_hold, f_max
        self.preshape_time, self.close_time = preshape_time, close_time
        self.settle_time, self.backoff = settle_time, backoff
        self.grip: Grip | None = None
        self.phase = "idle"
        self.t = 0.0
        self.progress = np.zeros(5)
        self.target = np.zeros(5)
        self._start = np.zeros(5)
        self._still = 0.0
        self.in_contact = np.zeros(5, bool)

    def command(self, name: str, current=None):
        """Start a grip (name in GRIPS). 'open' releases."""
        grip = GRIPS[name]
        self.grip = grip
        self._start = np.array(self.target if current is None else current, float)
        self.phase = "release" if name == "open" else "preshape"
        self.t = 0.0
        self.progress[:] = 0.0
        self._still = 0.0

    def step(self, dt: float, forces) -> tuple[np.ndarray, str]:
        """Advance dt seconds with the current fingertip forces (N, 5). Returns (target curl, phase)."""
        f = np.asarray(forces, float)
        self.in_contact = f >= self.f_hold
        if self.grip is None:
            return self.target.copy(), self.phase
        g = self.grip
        pre, close = np.array(g.preshape, float), np.array(g.close, float)
        self.t += dt
        if self.phase in ("preshape", "release"):
            a = min(self.t / self.preshape_time, 1.0)
            self.target = self._start + (pre - self._start) * a
            if a >= 1.0:
                self.phase = "idle" if self.phase == "release" else "closing"
                self.t = 0.0
        elif self.phase == "closing":
            rate = dt / self.close_time
            for i in range(5):
                if self.t < g.delay[i]:
                    continue
                if f[i] > self.f_max:
                    self.progress[i] = max(0.0, self.progress[i] - self.backoff * rate * 4)
                elif f[i] < self.f_hold:
                    self.progress[i] = min(1.0, self.progress[i] + rate)
            self.target = pre + (close - pre) * self.progress
            done = (self.progress >= 1.0 - 1e-9) | self.in_contact
            if self.t >= max(g.delay) and done.all():
                self._still += dt
                if self._still >= self.settle_time:
                    self.phase = "holding"
            else:
                self._still = 0.0
        elif self.phase == "holding":  # keep the squeeze bounded: back off if the force climbs
            for i in range(5):
                if f[i] > self.f_max:
                    self.progress[i] = max(0.0, self.progress[i] - self.backoff * dt / self.close_time)
            self.target = pre + (close - pre) * self.progress
        return self.target.copy(), self.phase


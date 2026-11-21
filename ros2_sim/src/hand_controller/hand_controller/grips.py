"""Grip types as coordinated finger poses, and a force-limited closing planner (ROS-free, numpy only).

A grip is two poses in curl space (little, index, middle, ring, thumb; 0 open .. 1 closed):

    preshape  hand shaped for the approach (fingers half open, thumb opposed ...)
    close     final pose when nothing is in the way

``GripPlanner`` runs  open/any -> preshape -> close -> hold. While closing, every finger advances towards its final pose
at its own pace (the thumb can start late) and **stops as soon as it touches something**: its fingertip force reached
``f_hold``, or it is *blocked* (its joint cannot follow the command: contact anywhere on the finger, which the tip sensor
does not see). If the force exceeds ``f_max`` it backs off a little. So a ball, a cup and a pen end up held by whichever
fingers touch them, with a bounded force (the simulated joint effort limit is the hard cap, this is the soft limit).

With ``verify=True`` (a grasp aimed at an object) the end of the closing is checked: at least ``Grip.min_contacts`` of the
fingers that should close are touching. If not the hand re-opens to the preshape and tries again (``max_attempts`` in all),
and the phase ends in ``failed`` (hand open) instead of ``holding`` so the caller never lifts an empty hand.
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
    min_contacts: int = 0       # fingers (of those that close) that must touch the object for the grasp to count as secure


GRIPS = {g.name: g for g in (
    Grip("open", (0, 0, 0, 0, 0), (0, 0, 0, 0, 0), description="all fingers extended"),
    Grip("point", (0.4, 0, 0.4, 0.4, 0.2), (1, 0, 1, 1, 0.6), (0, 0, 0.05, 0.1, 0.2),
         "index extended, the other fingers and the thumb folded"),
    Grip("pinch", (0, 0.3, 0, 0, 0.3), (0, 0.9, 0, 0, 0.5), (0, 0, 0, 0, 0),
         "index tip against the thumb tip (precision grip, e.g. a pen); the other fingers stay open", min_contacts=1),
    Grip("power", (0.3, 0.3, 0.3, 0.3, 0.3), (1, 1, 1, 1, 0.6), (0.2, 0.0, 0.0, 0.1, 1.0),
         "all fingers wrap the object, thumb closes last (ball, cup)", min_contacts=2),
)}

# tracker gesture -> grip, for grip-assist mode
GESTURE_TO_GRIP = {"Open Hand": "open", "Pointing": "point", "Pinch": "pinch", "Fist": "power"}

# --- automatic grip selection -------------------------------------------------------------------------------------
# Width of the objects of the grasp world (m): a thin object is held between the fingertips (pinch), a wide one is wrapped.
OBJECT_WIDTH = {"ball": 0.040, "cup": 0.050, "pen": 0.012}
THIN_MAX = 0.025          # <= this wide: pinch; wider: power
CLOSING_GESTURES = ("Fist", "Pinch")   # the hand pose that means "grab"


def select_grip(gesture: str, nearest=None, reach: float = 0.20, widths=OBJECT_WIDTH, thin_max: float = THIN_MAX):
    """Grip for the current hand pose and the nearest object, or None when the gesture asks for nothing.

    gesture   the tracker's gesture label (the hand pose);
    nearest   (object name, distance from the palm in m) or None.

    The hand pose says *when*: Open Hand releases, Fist / Pinch means "grab" (so does any other pose via GESTURE_TO_GRIP,
    e.g. Pointing). The nearest object within `reach` says *which*: a thin object (pen) -> pinch, a wide one (ball, cup) ->
    power grip, whichever of the two closing poses the user made. With no object in reach the gesture alone decides
    (Fist -> power, Pinch -> pinch), as in plain grip-assist.
    """
    if gesture not in CLOSING_GESTURES:
        return GESTURE_TO_GRIP.get(gesture)
    if nearest is not None and nearest[1] <= reach and nearest[0] in widths:
        return "pinch" if widths[nearest[0]] <= thin_max else "power"
    return GESTURE_TO_GRIP[gesture]


class GripPlanner:
    """phase: idle -> preshape -> closing -> holding;  'release' goes back to the open pose; 'failed' = verified grasp missed."""

    def __init__(self, f_hold=2.0, f_max=5.0, preshape_time=0.6, close_time=1.6, settle_time=0.3, backoff=0.25, max_attempts=3, regrip=False, regrip_rate=0.6):
        self.f_hold, self.f_max = f_hold, f_max
        self.preshape_time, self.close_time = preshape_time, close_time
        self.settle_time, self.backoff = settle_time, backoff
        self.max_attempts = max_attempts
        self.regrip, self.regrip_rate = regrip, regrip_rate
        self.grip: Grip | None = None
        self.phase = "idle"
        self.t = 0.0
        self.progress = np.zeros(5)
        self.target = np.zeros(5)
        self._start = np.zeros(5)
        self._still = 0.0
        self.verify = False
        self.attempt = 0
        self.in_contact = np.zeros(5, bool)

    def command(self, name: str, current=None, verify: bool = False):
        """Start a grip (name in GRIPS). 'open' releases. verify: check the grasp (see the module docstring)."""
        grip = GRIPS[name]
        self.grip = grip
        self._start = np.array(self.target if current is None else current, float)
        self.phase = "release" if name == "open" else "preshape"
        self.t = 0.0
        self.progress[:] = 0.0
        self._still = 0.0
        self.verify = verify and grip.min_contacts > 0
        self.attempt = 1

    def secure(self) -> bool:
        """Enough of the closing fingers touch the object?"""
        g = self.grip
        active = np.array(g.close) > np.array(g.preshape) + 0.05
        return int((self.in_contact & active).sum()) >= min(g.min_contacts, int(active.sum()))

    def step(self, dt: float, forces, blocked=None) -> tuple[np.ndarray, str]:
        """Advance dt seconds with the current fingertip forces (N, 5) and the per-finger 'blocked' flags (bool, 5).
        Returns (target curl, phase)."""
        f = np.asarray(forces, float)
        blk = np.zeros(5, bool) if blocked is None else np.asarray(blocked, bool)
        self.in_contact = (f >= self.f_hold) | blk
        if self.grip is None:
            return self.target.copy(), self.phase
        g = self.grip
        pre, close = np.array(g.preshape, float), np.array(g.close, float)
        self.t += dt
        if self.phase in ("preshape", "release", "failed"):
            a = min(self.t / self.preshape_time, 1.0)
            self.target = self._start + (pre - self._start) * a
            if a >= 1.0 and self.phase != "failed":     # 'failed' stays: the hand is open again, the caller was told
                self.phase = "idle" if self.phase == "release" else "closing"
                self.t = 0.0
        elif self.phase == "closing":
            rate = dt / self.close_time
            for i in range(5):
                if self.t < g.delay[i]:
                    continue
                if f[i] > self.f_max:
                    self.progress[i] = max(0.0, self.progress[i] - self.backoff * rate * 4)
                elif f[i] < self.f_hold and not blk[i]:
                    self.progress[i] = min(1.0, self.progress[i] + rate)
            self.target = pre + (close - pre) * self.progress
            done = (self.progress >= 1.0 - 1e-9) | self.in_contact
            if self.t >= max(g.delay) and done.all():
                self._still += dt
                if self._still >= self.settle_time:
                    self._finished_closing(pre)
            else:
                self._still = 0.0
        elif self.phase == "holding":
            # keep the squeeze bounded (back off if the force climbs) and keep the contact: a finger that was stopped by something
            # and is free now (the object slipped, or the table it pressed on is gone because the hand lifted) closes on
            # until it touches again (regrip=True; off by default: it did not rescue the ball and made the cup worse in the sim)
            active = close > pre + 0.05
            for i in range(5):
                if f[i] > self.f_max:
                    self.progress[i] = max(0.0, self.progress[i] - self.backoff * dt / self.close_time)
                elif self.regrip and active[i] and not self.in_contact[i]:
                    self.progress[i] = min(1.0, self.progress[i] + self.regrip_rate * dt / self.close_time)
            self.target = pre + (close - pre) * self.progress
        return self.target.copy(), self.phase

    def _finished_closing(self, pre):
        if not self.verify or self.secure():
            self.phase = "holding"
        elif self.attempt < self.max_attempts:     # missed: open back to the preshape and close again
            self.attempt += 1
            self._start, self.phase, self.t, self._still = self.target.copy(), "preshape", 0.0, 0.0
            self.progress[:] = 0.0
        else:                                      # give up: open the hand, report it
            self._start, self.phase, self.t = self.target.copy(), "failed", 0.0
            self.progress[:] = 0.0
            self.grip = GRIPS["open"]

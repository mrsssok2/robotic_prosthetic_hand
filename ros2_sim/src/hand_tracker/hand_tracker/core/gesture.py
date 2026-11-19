"""Rule-based gesture classification from per-finger curl (same 5 gestures as the original)."""
from __future__ import annotations

import numpy as np

NONE = "None"

# Pattern per finger in FINGER_ORDER (little, index, middle, ring, thumb):
# 'o' = must be open, 'c' = must be closed, 'x' = don't care.
GESTURES = {
    "Fist":      "cccc" "x",
    "Pointing":  "cocc" "x",
    "Peace":     "cooc" "x",
    "Open Hand": "oooo" "o",
    "Call Me":   "occc" "o",
}


def classify_gesture(curls, open_below: float = 0.35, closed_above: float = 0.6) -> str:
    """Return a gesture name or 'None'.

    A finger with curl between the two thresholds is "ambiguous" and fails any
    pattern that constrains it, which avoids flicker while a finger is mid-motion.
    """
    c = np.asarray(curls, dtype=float)
    if c.shape != (5,):
        raise ValueError(f"expected 5 curls, got {c.shape}")
    for name, pattern in GESTURES.items():
        ok = True
        for value, want in zip(c, pattern):
            if (want == "o" and value >= open_below) or (want == "c" and value <= closed_above):
                ok = False
                break
        if ok:
            return name
    return NONE

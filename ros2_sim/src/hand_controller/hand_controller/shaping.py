"""Smoothing, rate limiting and limits for the finger commands (ROS-free, numpy only)."""
import numpy as np

SERVO_MAX_DEG = 180.0


class OneEuroFilter:
    """One Euro filter (Casiez, Roussel, Vogel, CHI 2012), one independent filter per element, vectorised.

    An adaptive low-pass: the cut-off rises with the (filtered) speed of the signal, so a still finger is smoothed hard
    (low jitter) while a fast closing is hardly delayed (low lag).

        speed   = low-pass(d x / dt, d_cutoff)
        cutoff  = min_cutoff + beta * |speed|          [Hz]
        x_hat   = low-pass(x, cutoff)                  alpha = 1 / (1 + 1 / (2 pi cutoff dt))

    min_cutoff: jitter at rest (lower = smoother, laggier). beta: speed coefficient (higher = less lag in fast motion, more
    jitter there); both in the units of the signal (curl per second for beta). d_cutoff only smooths the speed estimate.
    A gap longer than `reset_after` seconds (tracking was lost) restarts from the new sample instead of blending.
    """

    def __init__(self, n, min_cutoff=1.0, beta=0.5, d_cutoff=1.0, reset_after=0.5, initial=None):
        self.min_cutoff, self.beta, self.d_cutoff, self.reset_after = min_cutoff, beta, d_cutoff, reset_after
        self.x = None if initial is None else np.full(n, float(initial))
        self.dx = np.zeros(n)
        self.n = n

    @staticmethod
    def _alpha(cutoff, dt):
        return 1.0 / (1.0 + 1.0 / (2.0 * np.pi * np.maximum(cutoff, 1e-9) * dt))

    def __call__(self, x, dt):
        x = np.asarray(x, float)
        if self.x is None or dt > self.reset_after or dt <= 0.0:
            self.x, self.dx = x.copy(), np.zeros(self.n)
            return self.x.copy()
        speed = (x - self.x) / dt
        self.dx += self._alpha(self.d_cutoff, dt) * (speed - self.dx)
        cutoff = self.min_cutoff + self.beta * np.abs(self.dx)
        self.x = self.x + self._alpha(cutoff, dt) * (x - self.x)
        return self.x.copy()


class Shaper:
    """[EMA smoothing] -> per-tick rate limit -> clamp to [lo, hi].

    alpha = 0.3 is the original project's EMA (kept for comparison, ``smoothing:=ema``); alpha = 1 disables it and leaves only
    the rate limit (the One Euro filter runs upstream, on the arriving samples). max_rate is in units per second, so
    4.0 curl/s means a full open->close takes at least 0.25 s.
    """

    def __init__(self, n, alpha=0.3, max_rate=4.0, initial=0.0, lo=0.0, hi=1.0):
        self.alpha, self.max_rate = alpha, max_rate
        self.lo, self.hi = np.broadcast_to(np.asarray(lo, float), (n,)), np.broadcast_to(np.asarray(hi, float), (n,))
        self.value = np.full(n, float(initial))

    def step(self, target, dt):
        target = np.clip(np.asarray(target, float), self.lo, self.hi)
        wanted = self.value + self.alpha * (target - self.value)
        limit = self.max_rate * dt
        self.value = np.clip(self.value + np.clip(wanted - self.value, -limit, limit), self.lo, self.hi)
        return self.value.copy()


def servo_angles(curl):
    """Curl (little, index, middle, ring, thumb) -> servo degrees, channels 0-4.

    Same convention as the original: fingers 0 deg open .. 180 closed, thumb reversed.
    """
    deg = np.asarray(curl, float) * SERVO_MAX_DEG
    deg[4] = SERVO_MAX_DEG - deg[4]
    return deg


# --- one value per finger -> joint angles (rad) ---------------------------------------------
# Must match the properties in hand_description/urdf/hand.urdf.xacro (test_urdf_sync.py checks it).
MCP_MAX, K_PIP, K_DIP = 1.57, 1.2, 0.7
# Thumb: all four joints follow the single thumb value; each reaches its maximum at curl = 1.
THUMB_MAX = {"thumb_swing": 2.95, "thumb_opp": 0.43, "thumb_mcp": 0.55, "thumb_ip": 0.5}

FINGERS = ("little", "index", "middle", "ring")  # curl[0:4]; curl[4] is the thumb

# Wrist (rad): pronation (+) / supination (-) and flexion (+) / extension (-); WRIST_* in the URDF.
WRIST_NAMES = ["wrist_pronation", "wrist_flexion"]
WRIST_LIMITS = np.array([[-1.4, 1.4], [-1.0, 1.0]])

# Order of the 18 joints commanded to finger_position_controller (see hand_bringup controllers.yaml):
# 12 finger joints, 4 thumb joints, 2 wrist joints.
JOINT_NAMES = [f"{f}_{j}" for f in FINGERS for j in ("mcp", "pip", "dip")] + list(THUMB_MAX) + WRIST_NAMES
_SCALE = np.array([s for _ in FINGERS for s in (MCP_MAX, MCP_MAX * K_PIP, MCP_MAX * K_DIP)] + list(THUMB_MAX.values()))
_PER_CURL = np.array([i // 3 if i < 12 else 4 for i in range(len(_SCALE))])
# URDF upper limits (a little above the commanded maximum); commands are clamped to [0, limit].
JOINT_LIMITS = _SCALE + 0.05


def joint_targets(curl, wrist=(0.0, 0.0)):
    """Curl (little, index, middle, ring, thumb) + wrist (pronation, flexion, rad) -> the 18 joint angles in JOINT_NAMES order.

    Finger tendons: PIP and DIP are fixed multiples of the MCP angle. Thumb: swing, opposition and
    both tip joints scale together, so closing the thumb sweeps it across the palm. The wrist is clamped to its limits.
    """
    unit = np.clip(np.asarray(curl, float), 0.0, 1.0)
    fingers = np.clip(unit[_PER_CURL] * _SCALE, 0.0, JOINT_LIMITS)
    return np.concatenate([fingers, np.clip(np.asarray(wrist, float), WRIST_LIMITS[:, 0], WRIST_LIMITS[:, 1])])


# Reach workspace in joint coordinates (m) of the grasp world: reach_x, reach_y, lift = HOME +- HALF.
# Lift >= -0.03 keeps the palm underside 2+ cm above the table top; x covers the table objects; y keeps the forearm
# (hanging behind the palm) clear of the base. Must stay inside the joint limits of the URDF (test_shaping checks).
REACH_HOME = np.array([0.0, -0.10, 0.06])
REACH_HALF = np.array([0.35, 0.15, 0.09])

"""Smoothing, rate limiting and limits for the finger commands (ROS-free, numpy only)."""
import numpy as np

SERVO_MAX_DEG = 180.0


class Shaper:
    """EMA smoothing -> per-tick rate limit -> clamp to [0, 1] (curl units).

    alpha = 0.3 is the original project's smoothing factor. max_rate is in curl/s,
    so 4.0 means a full open->close takes at least 0.25 s.
    """

    def __init__(self, n, alpha=0.3, max_rate=4.0, initial=0.0):
        self.alpha, self.max_rate = alpha, max_rate
        self.value = np.full(n, float(initial))

    def step(self, target, dt):
        target = np.clip(np.asarray(target, float), 0.0, 1.0)
        wanted = self.value + self.alpha * (target - self.value)
        limit = self.max_rate * dt
        self.value = np.clip(self.value + np.clip(wanted - self.value, -limit, limit), 0.0, 1.0)
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

# Order of the 16 joints commanded to finger_position_controller (see hand_bringup controllers.yaml).
JOINT_NAMES = [f"{f}_{j}" for f in FINGERS for j in ("mcp", "pip", "dip")] + list(THUMB_MAX)
_SCALE = np.array([s for _ in FINGERS for s in (MCP_MAX, MCP_MAX * K_PIP, MCP_MAX * K_DIP)] + list(THUMB_MAX.values()))
_PER_CURL = np.array([i // 3 if i < 12 else 4 for i in range(len(JOINT_NAMES))])
# URDF upper limits (a little above the commanded maximum); commands are clamped to [0, limit].
JOINT_LIMITS = _SCALE + 0.05


def joint_targets(curl):
    """Curl (little, index, middle, ring, thumb) -> the 16 joint angles in JOINT_NAMES order.

    Finger tendons: PIP and DIP are fixed multiples of the MCP angle. Thumb: swing, opposition and
    both tip joints scale together, so closing the thumb sweeps it across the palm.
    """
    unit = np.clip(np.asarray(curl, float), 0.0, 1.0)
    return np.clip(unit[_PER_CURL] * _SCALE, 0.0, JOINT_LIMITS)

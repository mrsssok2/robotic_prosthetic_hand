"""Synthetic MediaPipe-style (21, 3) hands with known joint flexions (tests + fake input)."""
import numpy as np

# finger -> (wrist->MCP x offset, bone lengths MCP-PIP, PIP-DIP, DIP-TIP)
_FINGERS = {
    "index": (5, -0.15, (0.40, 0.25, 0.20)),
    "middle": (9, 0.0, (0.45, 0.28, 0.22)),
    "ring": (13, 0.15, (0.40, 0.26, 0.20)),
    "little": (17, 0.30, (0.32, 0.20, 0.17)),
}


def _rot(v, axis, deg):
    """Rodrigues rotation of v about unit axis."""
    a = np.radians(deg)
    return v * np.cos(a) + np.cross(axis, v) * np.sin(a) + axis * np.dot(axis, v) * (1 - np.cos(a))


def _chain(start, d0, axis, lengths, flexions):
    """Points after `start`; bone k direction = d0 rotated by sum(flexions[:k+1])."""
    pts, p, total = [], np.asarray(start, float), 0.0
    for length, flex in zip(lengths, flexions):
        total += flex
        p = p + _rot(d0, axis, total) * length
        pts.append(p)
    return pts


def make_hand(flex_deg: dict) -> np.ndarray:
    """flex_deg: finger -> flexions (MCP,PIP,DIP deg); "thumb" -> sweep 0..1. Missing = straight/open."""
    pts = np.zeros((21, 3))
    for name, (base, x, lengths) in _FINGERS.items():
        mcp = np.array([x, 1.0, 0.0])
        d0 = mcp / np.linalg.norm(mcp)
        axis = np.cross(d0, [0, 0, 1.0])
        axis /= np.linalg.norm(axis)
        pts[base] = mcp
        for k, p in enumerate(_chain(mcp, d0, axis, lengths, flex_deg.get(name, (0, 0, 0)))):
            pts[base + 1 + k] = p
    # thumb: a straight 3-bone chain pivoting about its CMC in the palm plane.
    # t = 0 sticks out sideways (open), t = 1 lies across the palm toward the little finger.
    t = float(flex_deg.get("thumb", 0.0))
    phi = np.radians(127.0 * t)
    open_dir = np.array([-0.6, 0.8, 0.0])
    d = np.array([open_dir[0] * np.cos(phi) + open_dir[1] * np.sin(phi),
                  -open_dir[0] * np.sin(phi) + open_dir[1] * np.cos(phi), 0.0])
    pts[1] = np.array([-0.25, 0.25, 0.0])
    for k, length in enumerate((0.3, 0.25, 0.2)):
        pts[2 + k] = pts[1 + k] + d * length
    return pts


def pose(closed_fingers=()):
    """Hand where the listed fingers are fully curled and all others straight."""
    flex = {}
    for f in closed_fingers:
        flex[f] = (60, 80, 60) if f != "thumb" else 1.0
    return make_hand(flex)

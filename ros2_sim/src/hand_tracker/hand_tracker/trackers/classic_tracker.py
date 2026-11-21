"""Classic-CV baseline: skin segmentation -> largest blob -> palm centre -> radial profile -> open/closed per finger.

No learning, numpy + OpenCV only. Pipeline
  1. YCrCb skin threshold, morphology, largest contour.
  2. Palm centre / radius R = maximum of the distance transform of the blob.
  3. Radial extent r(theta) of the blob around the palm centre (capped at CAP*R). A finger sticking out is
     a narrow peak with r > OPEN_R * R. The forearm is a wide span that reaches the cap and is dropped; its
     direction gives the "wrist" direction (otherwise: opposite to the mean fingertip direction).
  4. Peaks are assigned to finger slots by their angle from the wrist axis (middle at 0 deg, index/thumb on
     one side, ring/little on the other; both mirror choices are tried and the better fit wins).
     A slot with a peak = open (curl 0.0..0.5 by how far the tip reaches), an empty slot = closed (curl 1).

Known weak spots (this is the baseline, not the product): colour thresholds break with lighting and
background, a bare forearm can steal the palm centre, the thumb is only seen when it sticks out, and
there are no keypoints, so there is no keypoint error to report. Convexity defects (valleys between the
fingers) are used to reject peaks that are just bumps of the hand outline.
"""
import cv2
import numpy as np

from .base import HandResult, Tracker

CAP = 3.3          # radial cap (x R); the arm saturates here, fingers do not
OPEN_R = 1.35
THUMB_OPEN_R = 1.2
# angle of each finger slot from the wrist->middle axis, deg (one side positive); order little, index, middle, ring, thumb
SLOTS = np.array([-30.0, 14.0, 0.0, -12.0, 58.0])
GATE = np.array([18.0, 16.0, 14.0, 14.0, 30.0])


def skin_mask(bgr):
    ycc = cv2.cvtColor(cv2.GaussianBlur(bgr, (5, 5), 0), cv2.COLOR_BGR2YCrCb)
    m = cv2.inRange(ycc, (0, 135, 85), (255, 180, 135))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)))


def radial_profile(mask, center, R, step=3):
    """r(theta) for theta = 0, step, ... : distance along the ray until the blob ends (capped)."""
    h, w = mask.shape
    thetas = np.deg2rad(np.arange(0, 360, step))
    rs = np.arange(0, CAP * R, max(1.0, R / 25))
    xs = (center[0] + np.cos(thetas)[:, None] * rs[None]).round().astype(int)
    ys = (center[1] + np.sin(thetas)[:, None] * rs[None]).round().astype(int)
    inside = (xs >= 0) & (xs < w) & (ys >= 0) & (ys < h)
    hit = np.zeros(xs.shape, bool)
    hit[inside] = mask[ys[inside], xs[inside]] > 0
    out = np.zeros(len(thetas))
    for i in range(len(thetas)):
        miss = np.where(~hit[i])[0]
        if not len(miss):
            out[i] = rs[-1]
        elif miss[0] == 0:
            out[i] = 0.0
        elif not inside[i, miss[0]]:
            out[i] = rs[-1]              # ray left the frame while still inside the blob: forearm cut by the border
        else:
            out[i] = rs[miss[0] - 1]
    return thetas, out


def _spans(above):
    """Circular runs of True in a boolean array -> list of index arrays."""
    n = len(above)
    if above.all():
        return [np.arange(n)]
    start = int(np.argmin(above))            # first False; rotate so the array starts on a False
    idx = (np.arange(n) + start) % n
    runs, cur = [], []
    for i in idx:
        if above[i]:
            cur.append(i)
        elif cur:
            runs.append(np.array(cur)); cur = []
    if cur:
        runs.append(np.array(cur))
    return runs


def _assign(phis, slot_ang, gate):
    """Order-preserving assignment of sorted tip angles to sorted finger slots (brute force, <= 5 tips).
    Cost = sum of |angle error| / gate, plus 1.5 per tip left without a slot. Returns (cost, [(tip, slot), ...])."""
    from itertools import combinations
    best = (1.5 * len(phis), [])
    for m in range(1, min(len(phis), len(slot_ang)) + 1):
        for tip_idx in combinations(range(len(phis)), m):
            for slot_idx in combinations(range(len(slot_ang)), m):
                c = sum(abs(phis[t] - slot_ang[k]) / gate[k] for t, k in zip(tip_idx, slot_idx)) + 1.5 * (len(phis) - m)
                if c < best[0]:
                    best = (c, list(zip(tip_idx, slot_idx)))
    return best


class ClassicTracker(Tracker):
    name = "classic"

    def __init__(self, min_area_frac=0.01, **_):
        self.min_area_frac = min_area_frac

    def process(self, bgr):
        h, w = bgr.shape[:2]
        scale = 320.0 / max(h, w) if max(h, w) > 320 else 1.0
        small = cv2.resize(bgr, None, fx=scale, fy=scale) if scale != 1.0 else bgr
        mask = skin_mask(small)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        cnt = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(cnt) < self.min_area_frac * mask.size:
            return None
        blob = np.zeros_like(mask)
        cv2.drawContours(blob, [cnt], -1, 255, -1)
        dist = cv2.distanceTransform(blob, cv2.DIST_L2, 5)
        _, R, _, c = cv2.minMaxLoc(dist)
        if R < 4:
            return None
        thetas, r = radial_profile(blob, c, R)
        deg = np.rad2deg(thetas)
        rr = r / R
        runs = _spans(rr > 1.1)
        peaks, arms = [], []             # peaks: (angle deg, reach in R); arms: (width, angle) of capped wide spans
        for run in runs:
            width = len(run) * 3.0
            if rr[run].max() >= CAP - 0.05 and width > 40:
                arms.append((width, deg[run][len(run) // 2]))
            elif width > 40:             # wide but not capped: several fingers side by side
                k = max(2, int(round(width / 13.0)))
                for j in range(k):       # one finger per ~13 deg slice; reach = longest point of the slice
                    part = run[int(j * len(run) / k):int((j + 1) * len(run) / k)]
                    peaks.append((deg[part][len(part) // 2], rr[part].max()))
            else:
                j = int(np.argmax(rr[run]))
                peaks.append((deg[run][j], rr[run][j]))
        arm = max(arms)[1] if arms else None
        # forearm leaving the frame: the blob pixels on the image border point from the palm towards the wrist
        edge = np.concatenate([blob[0, :], blob[-1, :], blob[:, 0], blob[:, -1]])
        ys, xs = np.where(blob > 0)
        bx = np.concatenate([np.where(blob[0, :] > 0)[0], np.where(blob[-1, :] > 0)[0],
                             np.zeros(int((blob[:, 0] > 0).sum())), np.full(int((blob[:, -1] > 0).sum()), blob.shape[1] - 1)])
        by = np.concatenate([np.zeros(int((blob[0, :] > 0).sum())), np.full(int((blob[-1, :] > 0).sum()), blob.shape[0] - 1),
                             np.where(blob[:, 0] > 0)[0], np.where(blob[:, -1] > 0)[0]])
        if len(bx) >= 8:
            arm = float(np.rad2deg(np.arctan2(by.mean() - c[1], bx.mean() - c[0])))
        peaks = [p for p in peaks if p[1] >= THUMB_OPEN_R]
        if arm is None:
            if not peaks:
                return self._result(np.ones(5), 0.5)
            mean = np.rad2deg(np.arctan2(np.mean([np.sin(np.deg2rad(a)) for a, _ in peaks]),
                                         np.mean([np.cos(np.deg2rad(a)) for a, _ in peaks])))
            arm = mean + 180.0
        axis = arm + 180.0               # forward = away from the wrist
        # a finger cannot point back towards the wrist: such peaks are forearm/shirt bumps
        peaks = [p for p in peaks if abs((p[0] - axis + 180.0) % 360.0 - 180.0) <= 110.0]
        self.debug = dict(center=c, R=R, peaks=peaks, arm=arm, axis=axis, scale=scale)

        best = None
        tips = sorted(peaks, key=lambda p: (p[0] - axis + 180.0) % 360.0 - 180.0)
        phis = [(a - axis + 180.0) % 360.0 - 180.0 for a, _ in tips]
        for s in (1.0, -1.0):
            slot_ang = s * SLOTS
            order = np.argsort(slot_ang)                      # slots left -> right
            cost, assign = _assign(phis, slot_ang[order], GATE[order])
            if best is None or cost < best[0]:
                best = (cost, {int(order[j]): tips[i][1] for i, j in assign})
        curl = np.ones(5)
        for k, reach in best[1].items():
            need = THUMB_OPEN_R if k == 4 else OPEN_R
            if reach >= need:
                curl[k] = float(np.clip(0.45 - 0.45 * (reach - need), 0.0, 0.45))
        return self._result(curl, 1.0)

    @staticmethod
    def _result(curl, score):
        return HandResult(np.asarray(curl, float), None, score, {"open": curl < 0.5})

"""Hand geometry checks on the real URDF (forward kinematics) against the capsule model in scripts/dims.py:
 - the xacro joint positions agree with dims.py (which also generates the meshes),
 - proportions: middle finger longest, little shortest,
 - thumb: low on the palm's side, open pose points sideways ~70 deg from the index finger, closing swings it across
   the palm and it never passes through the palm or the fingers.
Needs xacro (ROS); skipped otherwise."""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(__file__)
sys.path[:0] = [HERE, os.path.join(HERE, "..", "scripts"), os.path.join(HERE, "..", "..", "hand_controller")]
xacro = pytest.importorskip("xacro")
import dims  # noqa: E402
from hand_fk import Hand  # noqa: E402
from hand_controller.shaping import JOINT_NAMES, joint_targets  # noqa: E402

URDF = os.path.join(HERE, "..", "urdf", "hand.urdf.xacro")
MARGIN = 0.001  # required clearance between surfaces (m)
FINGER_NAMES = ("little", "ring", "middle", "index")
Z = np.array([0.0, 0.0, 1.0])


@pytest.fixture(scope="module")
def hand():
    return Hand(xacro.process_file(URDF, mappings={"sim_gazebo": "true"}).toxml())


def pose(hand, curls):
    return hand.poses(dict(zip(JOINT_NAMES, joint_targets(curls))))


def capsules(hand, poses, links):
    """(start, end, radius) in the world frame for the given link names."""
    out = []
    for link in links:
        R, p = poses[link]
        part, idx = link_info(link)
        length, (r0, r1) = part
        out.append((p, p + R @ (Z * length), 0.5 * (r0 + r1)))
    return out


def link_info(link):
    name, kind = link.rsplit("_", 1)
    i = {"proximal": 0, "middle": 1, "distal": 2, "prox": 1, "dist": 2, "meta": 0}[kind]
    if name == "thumb":
        return (dims.THUMB["lengths"][i], dims.THUMB["radii"][i]), i
    return (dims.FINGERS[name]["lengths"][i], dims.segment_radii(name)[i]), i


FINGER_LINKS = [f"{f}_{s}" for f in FINGER_NAMES for s in ("proximal", "middle", "distal")]
THUMB_LINKS = ["thumb_meta", "thumb_prox", "thumb_dist"]


def points(cap, n=25, skip=0.0):
    a, b, r = cap
    t = np.linspace(0, 1, n)
    t = t[t * np.linalg.norm(b - a) >= skip]
    return a + np.outer(t, b - a), r


def palm_sdf(pts):
    h, rr, c = np.array(dims.PALM["half"]), dims.PALM["round"], np.array(dims.PALM["center"])
    q = np.abs(pts - c) - (h - rr)
    return np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0) - rr


def clearance(hand, thumb, fingers):
    """Smallest surface-to-surface distance between the thumb and the palm / fingers."""
    poses = pose(hand, [fingers] * 4 + [thumb])
    worst = (np.inf, "")
    palm_origin = poses["palm"][1]
    for cap in capsules(hand, poses, THUMB_LINKS):
        # the metacarpal's first 12 mm is the CMC ball sitting on the thenar pad, so it is skipped vs the palm
        skip = 0.012 if cap is capsules(hand, poses, THUMB_LINKS)[0] else 0.0
        pts, r = points(cap, skip=skip)
        d = (palm_sdf(pts - palm_origin) - r).min()
        worst = min(worst, (d, "palm"))
        for fcap, fl in zip(capsules(hand, poses, FINGER_LINKS), FINGER_LINKS):
            fp, fr = points(fcap)
            dist = np.linalg.norm(pts[:, None] - fp[None], axis=2).min()
            worst = min(worst, (dist - r - fr, fl))
    return worst


def test_urdf_matches_dims(hand):
    j = hand.joints
    for name, f in dims.FINGERS.items():
        assert j[f"{name}_mcp"]["xyz"] == pytest.approx([f["x"], 0, dims.PALM_H])
        assert j[f"{name}_pip"]["xyz"][2] == pytest.approx(f["lengths"][0])
        assert j[f"{name}_dip"]["xyz"][2] == pytest.approx(f["lengths"][1])
    t = dims.THUMB
    assert j["thumb_mount_fixed"]["xyz"] == pytest.approx(t["mount"])
    assert j["thumb_opp"]["rpy"][1] == pytest.approx(t["tilt"], abs=1e-4)
    assert j["thumb_mcp"]["xyz"][2] == pytest.approx(t["lengths"][0])
    assert j["thumb_ip"]["xyz"][2] == pytest.approx(t["lengths"][1])
    assert j["base"]["xyz"][2] == pytest.approx(dims.BASE_Z)


def test_finger_proportions():
    total = {n: sum(f["lengths"]) for n, f in dims.FINGERS.items()}
    assert max(total, key=total.get) == "middle" and min(total, key=total.get) == "little"
    assert total["ring"] > total["little"] and total["index"] > total["little"]


def test_thumb_open_pose_points_sideways_from_low_on_the_palm(hand):
    poses = pose(hand, [0] * 5)
    R, p = poses["thumb_meta"]
    d = R @ Z
    from_vertical = np.degrees(np.arctan2(d[0], d[2]))
    assert 60 <= from_vertical <= 80, from_vertical            # 60-80 deg from the index finger
    assert abs(d[1]) < 0.1                                       # in the palm plane, not sticking forward
    assert p[0] > 0.04                                           # on the palm's (+X) side
    assert p[2] - poses["palm"][1][2] < 0.4 * dims.PALM_H        # low on the palm


def test_closing_sweeps_the_thumb_across_the_palm_under_the_fingers(hand):
    tips = []
    for t in np.linspace(0, 1, 11):
        poses = pose(hand, [0, 0, 0, 0, t])
        a, b, _ = capsules(hand, poses, ["thumb_dist"])[0]
        tips.append(b - poses["palm"][1])
    x = np.array([tip[0] for tip in tips])
    assert np.all(np.diff(x[:-1]) < 0)                 # steadily sideways, toward the little finger (the last
                                                       # step only shortens the reach as the tip joints curl)
    assert x[0] > 0.06                                 # open: well out beside the palm
    assert x[-1] < -0.025                              # closed: across the palm, near the little finger
    assert tips[-1][2] < 0.04                          # ... low, under the curled fingertips (knuckles at 0.085)
    assert tips[-1][1] < -0.02                         # ... in front of the palm face


@pytest.mark.parametrize("thumb", np.linspace(0, 1, 11))
def test_no_clipping_with_open_fingers(hand, thumb):
    d, who = clearance(hand, thumb, 0.0)
    assert d > MARGIN, f"thumb={thumb:.1f}: thumb comes within {d*1000:.1f} mm of {who}"


@pytest.mark.parametrize("thumb", [0.0, 0.85, 0.9, 0.95, 1.0])
def test_no_clipping_in_a_fist(hand, thumb):
    """Fingers fully curled. (Passing through the middle of the sweep with the fingers already curled is
    not covered: one thumb value and the finger values move independently.)"""
    d, who = clearance(hand, thumb, 1.0)
    assert d > MARGIN, f"thumb={thumb:.2f}: thumb comes within {d*1000:.1f} mm of {who}"

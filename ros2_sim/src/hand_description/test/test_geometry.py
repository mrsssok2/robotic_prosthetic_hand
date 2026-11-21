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


def pose(hand, curls, wrist=(0.0, 0.0)):
    return hand.poses(dict(zip(JOINT_NAMES, joint_targets(curls, wrist))))


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
    assert j["base"]["xyz"][2] == pytest.approx(dims.BASE_Z - dims.WRIST_PIVOT)   # forearm frame = wrist pivot


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


# ===== wrist (pronation / flexion): the hand must not cut into the forearm over the whole range ====================
FOREARM_TOP = max(dims.FOREARM["cuff"]["z"][1], 0.0)   # forearm frame: the pivot is the centre of the shell's top face
WRIST_GRID = [(p, f) for p in (-1.4, -0.7, 0.0, 0.7, 1.4) for f in (-1.0, -0.5, 0.0, 0.6, 1.0)]


def forearm_clearance(local, r=0.0):
    """Approximate distance (m, negative = inside) from points (N, 3) in the forearm frame to the forearm + cuff.
    Radial: elliptical section whose semi-axes grow linearly down the shell, the cuff on top of it; above the cuff top
    a point is outside. Bottom of the shell is ignored (nothing reaches it)."""
    z = np.clip(local[:, 2], -dims.FOREARM["length"], FOREARM_TOP)
    t = np.clip(-z / dims.FOREARM["length"], 0, 1)
    a = dims.FOREARM["wrist_ab"][0] + t * (dims.FOREARM["bottom_ab"][0] - dims.FOREARM["wrist_ab"][0])
    b = dims.FOREARM["wrist_ab"][1] + t * (dims.FOREARM["bottom_ab"][1] - dims.FOREARM["wrist_ab"][1])
    c0, c1 = dims.FOREARM["cuff"]["z"]
    in_cuff = (local[:, 2] >= c0) & (local[:, 2] <= c1)
    a = np.where(in_cuff, np.maximum(a, dims.FOREARM["cuff"]["ab"][0]), a)
    b = np.where(in_cuff, np.maximum(b, dims.FOREARM["cuff"]["ab"][1]), b)
    radial = (np.sqrt((local[:, 0] / a) ** 2 + (local[:, 1] / b) ** 2) - 1.0) * np.minimum(a, b)
    return np.maximum(radial, local[:, 2] - FOREARM_TOP) - r


def palm_surface_points(n=7):
    h, c = np.array(dims.PALM["half"]), np.array(dims.PALM["center"])
    g = np.linspace(-1, 1, n)
    pts = [c + h * np.array(p) for p in ((x, y, z) for x in g for y in g for z in g) if max(abs(v) for v in p) == 1]
    return np.array(pts)


def hand_points(hand, poses):
    """(points in the world, radius) of the palm surface and every finger / thumb capsule."""
    R, p = poses["palm"]
    out = [(palm_surface_points() @ R.T + p, 0.0)]
    for cap in capsules(hand, poses, FINGER_LINKS + THUMB_LINKS):
        pts, r = points(cap, n=12)
        out.append((pts, r))
    return out


def worst_forearm_clearance(hand, curls, wrist):
    poses = pose(hand, curls, wrist)
    R, p = poses["forearm"]
    return min(forearm_clearance((pts - p) @ R, r).min() for pts, r in hand_points(hand, poses))


def test_wrist_joints_exist_with_the_right_axes_and_limits(hand):
    j = hand.joints
    assert j["wrist_pronation"]["parent"] == "forearm" and j["wrist_flexion"]["parent"] == "wrist_roll_link"
    assert j["wrist_pronation"]["axis"] == pytest.approx([0, 0, -1])      # + = pronation
    assert j["wrist_flexion"]["axis"] == pytest.approx([1, 0, 0])         # + = flexion, same direction as the fingers
    assert j["wrist_pronation"]["limit"] == pytest.approx((-1.4, 1.4))
    assert j["wrist_flexion"]["limit"] == pytest.approx((-1.0, 1.0))
    assert j["wrist_palm"]["xyz"][2] == pytest.approx(dims.WRIST_PIVOT)         # palm origin above the pivot
    assert j["wrist_flexion"]["xyz"] == pytest.approx([0, 0, 0])               # both axes through the pivot


def test_wrist_at_zero_leaves_the_palm_where_it_was(hand):
    R, p = pose(hand, [0] * 5)["palm"]
    np.testing.assert_allclose(R, np.eye(3), atol=1e-12)
    assert p[2] == pytest.approx(dims.BASE_Z)


def test_flexion_tips_the_fingers_toward_the_palm_side_and_pronation_turns_about_the_forearm(hand):
    R, _ = pose(hand, [0] * 5, (0.0, 0.8))["palm"]
    assert (R @ Z)[1] < -0.5                      # fingers now point toward -Y, the side the palm faces
    R, _ = pose(hand, [0] * 5, (0.8, 0.0))["palm"]
    np.testing.assert_allclose(R @ Z, Z, atol=1e-12)                 # the finger axis is the roll axis
    assert (R @ np.array([1.0, 0, 0]))[1] < -0.5  # the thumb side turns toward -Y with positive pronation (about -Z)


@pytest.mark.parametrize("curls", [[0] * 5, [1] * 5], ids=["open", "fist"])
@pytest.mark.parametrize("wrist", WRIST_GRID, ids=lambda w: f"pron{w[0]:+.1f}_flex{w[1]:+.1f}")
def test_wrist_never_cuts_into_the_forearm(hand, curls, wrist):
    d = worst_forearm_clearance(hand, curls, wrist)
    assert d > MARGIN, f"wrist={wrist}: the hand comes within {d*1000:.1f} mm of the forearm/cuff"


def test_wrist_ball_sits_inside_the_forearm_top_face():
    """The dome is a sphere about the pivot: it never touches the forearm as the wrist moves if it fits on the top face."""
    assert dims.WRIST_BALL["radius"] <= min(dims.FOREARM["wrist_ab"]) - 0.001
    assert dims.WRIST_BALL["neck_radius"] < dims.WRIST_BALL["radius"]


def test_wrist_limits_leave_the_palm_edges_above_the_forearm_in_theory():
    """The palm's lower edge (half thickness t) dips t*sin(angle) below the palm base about the pivot; it stays above the
    forearm's top face while WRIST_PIVOT > t * tan(angle)."""
    t = dims.PALM["half"][1]
    for lim in (1.0, 1.0):
        assert dims.WRIST_PIVOT > t * np.tan(lim)

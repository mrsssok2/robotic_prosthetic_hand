import numpy as np
import pytest

from hand_tracker.core import FINGER_ORDER, classify_gesture, finger_curls, joint_flexions, thumb_sweep
from hand_tracker.core.synthetic import make_hand, pose

ALL = ("little", "index", "middle", "ring", "thumb")


def random_rotation(rng):
    q, _ = np.linalg.qr(rng.normal(size=(3, 3)))
    return q * np.sign(np.linalg.det(q))  # proper rotation


def test_flexions_match_construction():
    flex = {"index": (10, 40, 25)}
    got = joint_flexions(make_hand(flex))
    np.testing.assert_allclose(got["index"], flex["index"], atol=1e-6)
    np.testing.assert_allclose(got["middle"], 0, atol=1e-6)


def test_open_and_closed_extremes():
    np.testing.assert_allclose(finger_curls(pose()), 0.0)
    np.testing.assert_allclose(finger_curls(pose(ALL)), 1.0)


def test_curl_is_monotonic_and_proportional():
    prev = -1.0
    for t in np.linspace(0, 1, 11):
        hand = make_hand({"middle": (60 * t, 80 * t, 60 * t)})
        c = finger_curls(hand)[FINGER_ORDER.index("middle")]
        assert c >= prev
        prev = c
    half = make_hand({"middle": (30, 40, 30)})  # total 100 deg, range 20..200
    assert finger_curls(half)[2] == pytest.approx((100 - 20) / 180)


def test_other_fingers_unaffected():
    c = finger_curls(pose(("ring",)))
    assert c[FINGER_ORDER.index("ring")] == 1.0
    assert np.delete(c, FINGER_ORDER.index("ring")).sum() == 0.0


@pytest.mark.parametrize("seed", range(5))
def test_rotation_scale_translation_and_mirror_invariance(seed):
    rng = np.random.default_rng(seed)
    hand = make_hand({"index": (30, 50, 30), "thumb": 0.6, "little": (60, 80, 40)})
    base = finger_curls(hand)
    moved = hand @ random_rotation(rng).T * rng.uniform(0.1, 3) + rng.normal(size=3)
    np.testing.assert_allclose(finger_curls(moved), base, atol=1e-9)
    mirrored = hand * np.array([-1, 1, 1])  # right hand <-> left hand
    np.testing.assert_allclose(finger_curls(mirrored), base, atol=1e-9)


def test_degenerate_input_does_not_crash():
    c = finger_curls(np.zeros((21, 3)))
    assert np.all(np.isfinite(c)) and np.all((0 <= c) & (c <= 1))


def test_bad_shape_rejected():
    with pytest.raises(ValueError):
        finger_curls(np.zeros((20, 3)))


@pytest.mark.parametrize("closed, expected", [
    (ALL, "Fist"),
    (("little", "middle", "ring"), "Pointing"),
    (("little", "ring"), "Peace"),
    ((), "Open Hand"),
    (("index", "middle", "ring"), "Call Me"),
    (("index", "middle", "ring", "little", "thumb"), "Fist"),
    (("little",), "None"),
])
def test_gestures_from_pose(closed, expected):
    assert classify_gesture(finger_curls(pose(closed))) == expected


def test_fist_ignores_thumb_but_open_hand_does_not():
    assert classify_gesture([1, 1, 1, 1, 0.0]) == "Fist"
    assert classify_gesture([0, 0, 0, 0, 1.0]) == "None"


def test_ambiguous_finger_gives_none():
    assert classify_gesture([0.5, 0, 0, 0, 0]) == "None"


def test_thumb_sweeps_sideways_across_the_palm():
    sweeps = [thumb_sweep(make_hand({"thumb": t})) for t in np.linspace(0, 1, 11)]
    assert all(a < b for a, b in zip(sweeps, sweeps[1:]))  # strictly monotonic
    assert sweeps[0] < -0.5 and sweeps[-1] > 0.6
    curls = [finger_curls(make_hand({"thumb": t}))[4] for t in np.linspace(0, 1, 11)]
    assert curls[0] == 0.0 and curls[-1] == 1.0
    assert all(a <= b for a, b in zip(curls, curls[1:]))


def test_thumb_flexing_in_depth_does_not_count():
    """Only the sideways position matters (like the original's x test): bending the tip toward
    the camera/palm normal must not change the value."""
    hand = make_hand({"thumb": 0.5})
    moved = hand.copy()
    moved[4, 2] += 0.3
    assert thumb_sweep(moved) == pytest.approx(thumb_sweep(hand))


def test_is_pinch():
    import numpy as np
    from hand_tracker.core import is_pinch
    kp = np.zeros((21, 2)); kp[5] = (0, 0); kp[17] = (1, 0)
    kp[4] = (0.2, 0.5); kp[8] = (0.3, 0.5)
    assert is_pinch(kp)
    assert is_pinch(kp, [0.1, 0.6, 0.1, 0.1, 0.5])           # index may be curled
    assert not is_pinch(kp, [0.9, 0.6, 0.9, 0.9, 0.5])       # fist with the thumb on the index
    kp[8] = (0.9, 0.5)
    assert not is_pinch(kp)

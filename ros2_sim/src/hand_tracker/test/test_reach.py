import numpy as np
import pytest

from hand_tracker.core.reach import ReachMapper, deadzone, hand_size, palm_centre


def hand(cx, cy, size):
    kp = np.zeros((21, 2))
    kp[0] = (cx, cy + size)          # wrist below the knuckles (fingers up)
    for i in (5, 9, 13, 17):
        kp[i] = (cx, cy)
    return kp


def test_helpers():
    kp = hand(300, 200, 100)
    assert hand_size(kp) == pytest.approx(100)
    np.testing.assert_allclose(palm_centre(kp), [300, 200 + 100 / 5])


def test_first_call_recentres_and_gives_zero():
    m = ReachMapper(640, 480)
    np.testing.assert_allclose(m.map(hand(320, 240, 100)), 0)


def test_directions():
    m = ReachMapper(640, 480, dead=0.0)
    m.map(hand(320, 240, 100))
    assert m.map(hand(420, 240, 100))[0] > 0.5       # right -> x +
    assert m.map(hand(320, 140, 100))[1] > 0.5       # up in the image -> y +
    assert m.map(hand(320, 240, 130))[2] > 0.3       # bigger = closer -> closeness +
    assert m.map(hand(320, 240, 70))[2] < -0.3


def test_saturates_and_recentres():
    m = ReachMapper(640, 480)
    m.map(hand(320, 240, 100))
    np.testing.assert_allclose(m.map(hand(620, 20, 400)), [1, 1, 1])
    m.recentre(hand(500, 100, 150))
    np.testing.assert_allclose(m.map(hand(500, 100, 150)), 0)


def test_dead_zone_ignores_small_motion_and_is_continuous():
    assert deadzone(0.05, 0.1) == 0
    assert deadzone(0.1, 0.1) == 0 and deadzone(1.0, 0.1) == pytest.approx(1.0)
    assert deadzone(-0.55, 0.1) == pytest.approx(-0.5)
    m = ReachMapper(640, 480, dead=0.1)
    m.map(hand(320, 240, 100))
    np.testing.assert_allclose(m.map(hand(325, 238, 101)), 0)

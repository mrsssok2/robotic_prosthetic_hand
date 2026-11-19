import numpy as np
import pytest

from hand_controller.shaping import JOINT_LIMITS, JOINT_NAMES, Shaper, joint_targets, servo_angles


def test_ema_matches_original_formula():
    s = Shaper(1, alpha=0.3, max_rate=1e9)
    assert s.step([1.0], 0.02)[0] == pytest.approx(0.3)
    assert s.step([1.0], 0.02)[0] == pytest.approx(0.3 + 0.3 * 0.7)


def test_rate_limit_caps_step():
    s = Shaper(1, alpha=1.0, max_rate=2.0)
    assert s.step([1.0], 0.1)[0] == pytest.approx(0.2)


def test_clamped_to_unit_range():
    s = Shaper(2, alpha=1.0, max_rate=1e9)
    np.testing.assert_allclose(s.step([5.0, -3.0], 0.02), [1.0, 0.0])


def test_converges():
    s = Shaper(1)
    for _ in range(200):
        v = s.step([0.7], 0.02)
    assert v[0] == pytest.approx(0.7, abs=1e-3)


def test_servo_convention_matches_original():
    np.testing.assert_allclose(servo_angles([0, 0, 0, 0, 0]), [0, 0, 0, 0, 180])  # open
    np.testing.assert_allclose(servo_angles([1, 1, 1, 1, 1]), [180, 180, 180, 180, 0])  # fist
    np.testing.assert_allclose(servo_angles([0.5] * 5), [90] * 5)


def test_joint_targets_extremes_and_limits():
    assert len(joint_targets([0] * 5)) == len(JOINT_NAMES) == 16
    np.testing.assert_allclose(joint_targets([0] * 5), 0)
    out = joint_targets([5] * 5)  # out-of-range input never exceeds the limits
    assert np.all(out <= JOINT_LIMITS) and np.all(out >= 0)


def test_one_thumb_value_drives_all_thumb_joints():
    idx = [JOINT_NAMES.index(n) for n in ("thumb_swing", "thumb_opp", "thumb_mcp", "thumb_ip")]
    half = joint_targets([0, 0, 0, 0, 0.5])
    full = joint_targets([0, 0, 0, 0, 1.0])
    np.testing.assert_allclose(half[idx], full[idx] / 2)
    assert np.all(half[:12] == 0)  # fingers untouched

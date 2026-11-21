import numpy as np
import pytest

from hand_controller.shaping import JOINT_LIMITS, JOINT_NAMES, WRIST_LIMITS, OneEuroFilter, Shaper, joint_targets, servo_angles


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
    assert len(joint_targets([0] * 5)) == len(JOINT_NAMES) == 18
    np.testing.assert_allclose(joint_targets([0] * 5), 0)
    out = joint_targets([5] * 5)  # out-of-range input never exceeds the limits
    assert np.all(out[:16] <= JOINT_LIMITS) and np.all(out[:16] >= 0)


def test_wrist_is_passed_through_and_clamped():
    out = joint_targets([0] * 5, wrist=(0.3, -0.4))
    np.testing.assert_allclose(out[-2:], [0.3, -0.4])
    assert JOINT_NAMES[-2:] == ["wrist_pronation", "wrist_flexion"]
    np.testing.assert_allclose(joint_targets([0] * 5, wrist=(9, 9))[-2:], WRIST_LIMITS[:, 1])
    np.testing.assert_allclose(joint_targets([0] * 5, wrist=(-9, -9))[-2:], WRIST_LIMITS[:, 0])
    np.testing.assert_allclose(joint_targets([1] * 5)[-2:], 0)   # the default wrist is neutral


def test_one_thumb_value_drives_all_thumb_joints():
    idx = [JOINT_NAMES.index(n) for n in ("thumb_swing", "thumb_opp", "thumb_mcp", "thumb_ip")]
    half = joint_targets([0, 0, 0, 0, 0.5])
    full = joint_targets([0, 0, 0, 0, 1.0])
    np.testing.assert_allclose(half[idx], full[idx] / 2)
    assert np.all(half[:12] == 0)  # fingers untouched


# --- One Euro filter ----------------------------------------------------------------------------------------------------
def run(flt, signal, n, dt=1 / 30):
    return np.array([flt(np.atleast_1d(signal(k * dt)), dt)[0] for k in range(n)])


def test_one_euro_passes_a_constant_unchanged():
    out = run(OneEuroFilter(1), lambda t: 0.6, 50)
    np.testing.assert_allclose(out, 0.6)


def test_one_euro_reduces_jitter_on_a_still_signal():
    noise = np.random.default_rng(0).normal(0, 0.03, 600)
    flt = OneEuroFilter(1, min_cutoff=0.5, beta=2.5)
    out = np.array([flt(np.array([0.5 + n]), 1 / 30)[0] for n in noise])
    assert out[60:].std() < 0.5 * noise.std()


def test_one_euro_beats_a_fixed_filter_with_the_same_resting_smoothing_on_lag():
    """Same resting cut-off: the adaptive filter follows a fast step much sooner than min_cutoff alone would."""
    step = lambda t: 0.0 if t < 0.5 else 1.0
    fast = run(OneEuroFilter(1, min_cutoff=0.5, beta=2.5), step, 60)
    slow = run(OneEuroFilter(1, min_cutoff=0.8, beta=0.0), step, 60)
    k_fast, k_slow = np.argmax(fast > 0.5), np.argmax(slow > 0.5)
    assert k_fast < k_slow
    assert k_fast - 15 <= 3          # within ~3 frames (100 ms) of the step at frame 15


def test_one_euro_filters_each_element_independently():
    flt = OneEuroFilter(3, min_cutoff=0.5, beta=2.5)
    for _ in range(30):
        out = flt(np.array([0.0, 1.0, 0.5]), 1 / 30)
    np.testing.assert_allclose(out, [0.0, 1.0, 0.5], atol=1e-6)
    out = flt(np.array([0.0, 0.0, 0.5]), 1 / 30)      # only the middle one moves
    assert out[0] == pytest.approx(0.0) and out[2] == pytest.approx(0.5) and out[1] < 1.0


def test_one_euro_restarts_after_a_gap():
    flt = OneEuroFilter(1)
    flt(np.array([0.0]), 1 / 30)
    assert flt(np.array([1.0]), 2.0)[0] == 1.0          # tracking was lost for 2 s: no blending with the old value


def test_one_euro_with_the_rate_limiter_stays_within_limits():
    flt, sh = OneEuroFilter(1), Shaper(1, alpha=1.0, max_rate=10.0)
    prev = 0.0
    for k in range(100):
        v = sh.step(flt(np.array([1.0 if k > 10 else 0.0]), 0.02), 0.02)[0]
        assert abs(v - prev) <= 10.0 * 0.02 + 1e-9 and 0.0 <= v <= 1.0
        prev = v


def test_shaper_per_element_limits():
    s = Shaper(2, alpha=1.0, max_rate=1e9, lo=[-1.0, 0.0], hi=[1.0, 2.0])
    np.testing.assert_allclose(s.step([-5.0, 5.0], 0.02), [-1.0, 2.0])

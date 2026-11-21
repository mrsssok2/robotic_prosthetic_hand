import numpy as np
import pytest

from hand_tracker.core.calibration import CAPTURE_FRAMES, Calibration, CaptureSession


def test_default_is_identity():
    c = Calibration()
    np.testing.assert_allclose(c.apply([0, 0.3, 0.5, 0.9, 1.0]), [0, 0.3, 0.5, 0.9, 1.0])


def test_maps_the_users_open_and_fist_to_0_and_1():
    c = Calibration(open=np.array([0.2, 0.1, 0.15, 0.2, 0.3]), fist=np.array([0.8, 0.7, 0.9, 0.8, 0.9]), has_open=True, has_fist=True)
    np.testing.assert_allclose(c.apply(c.open), 0)
    np.testing.assert_allclose(c.apply(c.fist), 1)
    np.testing.assert_allclose(c.apply((c.open + c.fist) / 2), 0.5)
    np.testing.assert_allclose(c.apply([0, 0, 0, 0, 0]), 0)           # below the open reading: clamped
    np.testing.assert_allclose(c.apply([1, 1, 1, 1, 1]), 1)


def test_a_finger_with_no_span_is_left_alone_and_the_others_still_calibrate():
    c = Calibration(open=np.array([0.2, 0.1, 0.1, 0.1, 0.1]), fist=np.array([0.25, 0.9, 0.9, 0.9, 0.9]))
    out = c.apply([0.25, 0.5, 0.5, 0.5, 0.5])
    assert out[0] == pytest.approx(0.25)                    # identity for the first finger
    assert out[1] == pytest.approx(0.5)


def test_wrist_neutral_is_subtracted():
    c = Calibration(wrist_neutral=np.array([0.1, -0.2]))
    np.testing.assert_allclose(c.apply_wrist([0.3, 0.1]), [0.2, 0.3])


def test_json_round_trip(tmp_path):
    c = Calibration(open=np.array([0.1, 0.2, 0.3, 0.2, 0.1]), fist=np.array([0.9] * 5), wrist_neutral=np.array([0.05, -0.1]),
                    has_open=True, has_fist=True)
    f = tmp_path / "cal.json"
    c.save(f)
    d = Calibration.load(f)
    np.testing.assert_allclose(d.open, c.open)
    np.testing.assert_allclose(d.fist, c.fist)
    np.testing.assert_allclose(d.wrist_neutral, c.wrist_neutral)
    assert d.has_open and d.has_fist


def test_missing_or_broken_file_gives_none(tmp_path):
    assert Calibration.load(tmp_path / "nope.json") is None
    (tmp_path / "bad.json").write_text("{not json")
    assert Calibration.load(tmp_path / "bad.json") is None
    (tmp_path / "wrong.json").write_text('{"open": [1, 2], "fist": [1]}')
    assert Calibration.load(tmp_path / "wrong.json") is None


def feed_pose(session, curl, wrist=(0.0, 0.0), n=CAPTURE_FRAMES, noise=0.01, seed=0):
    rng = np.random.default_rng(seed)
    done = False
    for k in range(n):
        done = session.feed(np.asarray(curl) + rng.normal(0, noise, 5), wrist, now=k / 30)
    return done


def test_capture_open_then_fist():
    s = CaptureSession()
    s.start("open", 0.0)
    assert feed_pose(s, [0.15] * 5, wrist=(0.1, 0.2))
    assert s.calibration.has_open and not s.active
    np.testing.assert_allclose(s.calibration.open, 0.15, atol=0.02)
    np.testing.assert_allclose(s.calibration.wrist_neutral, [0.1, 0.2], atol=1e-9)
    s.start("fist", 5.0)
    assert feed_pose(s, [0.85] * 5)
    assert s.calibration.has_fist and "saved" in s.message
    np.testing.assert_allclose(s.calibration.apply([0.15] * 5), 0, atol=0.05)
    np.testing.assert_allclose(s.calibration.apply([0.85] * 5), 1, atol=0.05)


def test_capture_waits_for_a_hand_and_times_out():
    s = CaptureSession()
    s.start("open", 0.0)
    assert not s.feed(None, None, 1.0) and s.active
    assert s.feed(None, None, 10.0) and not s.active and "failed" in s.message
    assert not s.calibration.has_open


def test_a_moving_hand_is_rejected():
    s = CaptureSession()
    s.start("fist", 0.0)
    for k in range(CAPTURE_FRAMES):
        s.feed([k / CAPTURE_FRAMES] * 5, (0, 0), k / 30)       # sweeping from 0 to 1
    assert not s.calibration.has_fist and "moved" in s.message


def test_reports_a_finger_that_did_not_change():
    s = CaptureSession()
    s.start("open", 0.0); feed_pose(s, [0.1] * 5)
    s.start("fist", 9.0); feed_pose(s, [0.9, 0.9, 0.12, 0.9, 0.9])
    assert "middle" in s.message


def test_reset():
    s = CaptureSession()
    s.start("open", 0.0); feed_pose(s, [0.3] * 5)
    s.reset()
    assert not s.calibration.has_open

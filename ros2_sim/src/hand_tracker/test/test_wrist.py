"""Wrist pronation / flexion from world landmarks, against hands built with known wrist angles."""
import numpy as np
import pytest

from hand_tracker.core import wrist_angles
from hand_tracker.core.wrist import FLEXION_LIMITS, PRONATION_LIMITS

# a flat right hand in the sim palm frame (x thumb side, y back of the hand, z fingers), metres
HAND = np.zeros((21, 3))
HAND[5] = [0.034, 0, 0.085]      # index MCP
HAND[9] = [0.0115, 0, 0.09]      # middle MCP
HAND[13] = [-0.0115, 0, 0.085]   # ring MCP
HAND[17] = [-0.034, 0, 0.07]     # little MCP
HAND[4] = [0.07, -0.01, 0.04]    # thumb tip
HAND[8] = [0.034, 0, 0.16]
HAND[12] = [0.0115, 0, 0.17]
NEUTRAL = np.column_stack([[1, 0, 0], [0, 0, 1], [0, -1, 0]])   # hand axes in camera frame: palm to the camera, fingers up


def rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def camera_view(pron, flex, mirrored=True, left=False, scale=1.0, offset=(0.0, 0.0, 0.3)):
    """World landmarks the camera would report for a hand with these wrist angles (pron + = pronation)."""
    rot = NEUTRAL @ rz(-pron) @ rx(flex)
    pts = HAND @ rot.T * scale + np.asarray(offset)
    if left:
        pts = pts * [-1, 1, 1]     # a left hand is the mirror image of a right hand
    if mirrored:
        pts = pts * [-1, 1, 1]     # hand_cam flips the frame for the selfie view
    return pts


def test_neutral_pose_is_zero():
    np.testing.assert_allclose(wrist_angles(camera_view(0, 0), "Right"), 0.0, atol=1e-9)


@pytest.mark.parametrize("pron", np.radians([-80, -45, -10, 0, 25, 60, 80]))
@pytest.mark.parametrize("flex", np.radians([-55, -20, 0, 30, 65]))
def test_recovers_known_angles(pron, flex):
    got = wrist_angles(camera_view(pron, flex), "Right")
    np.testing.assert_allclose(got, [pron, flex], atol=1e-9)


def test_invariant_to_scale_and_translation():
    a = wrist_angles(camera_view(0.5, -0.3), "Right")
    b = wrist_angles(camera_view(0.5, -0.3, scale=2.7, offset=(0.4, -0.2, 1.0)), "Right")
    np.testing.assert_allclose(a, b, atol=1e-9)


@pytest.mark.parametrize("mirrored", [True, False])
def test_left_hand_is_mirrored_into_a_right_hand(mirrored):
    """A left hand doing the same movement drives the (right) sim hand the same way."""
    label_right = "Right" if mirrored else "Left"      # MediaPipe swaps the label when the image is not mirrored
    label_left = "Left" if mirrored else "Right"
    right = wrist_angles(camera_view(0.6, 0.4, mirrored=mirrored), label_right, mirrored=mirrored)
    left = wrist_angles(camera_view(0.6, 0.4, mirrored=mirrored, left=True), label_left, mirrored=mirrored)
    np.testing.assert_allclose(right, [0.6, 0.4], atol=1e-9)
    np.testing.assert_allclose(left, right, atol=1e-9)


def test_signs_follow_the_camera_picture():
    # fingers tilted toward the camera with the palm facing it = the palm side is in front = flexion (+)
    toward = wrist_angles(camera_view(0, 0.5), "Right")[1]
    assert toward > 0


def test_degenerate_input_does_not_crash():
    out = wrist_angles(np.zeros((21, 3)), "Right")
    assert np.all(np.isfinite(out))


def test_limits_are_sane():
    assert PRONATION_LIMITS[0] < 0 < PRONATION_LIMITS[1] and FLEXION_LIMITS[0] < 0 < FLEXION_LIMITS[1]
    assert np.degrees(max(PRONATION_LIMITS[1], FLEXION_LIMITS[1])) < 95

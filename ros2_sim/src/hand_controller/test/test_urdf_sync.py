"""The joint ratios/limits in shaping.py must equal the ones in the URDF."""
import pathlib
import re

import numpy as np
import pytest

from hand_controller import shaping

XACRO = pathlib.Path(__file__).resolve().parents[2] / "hand_description/urdf/hand.urdf.xacro"
pytestmark = pytest.mark.skipif(not XACRO.exists(), reason="xacro not found (installed package)")


def prop(name):
    return float(re.search(rf'name="{name}" value="([0-9.]+)"', XACRO.read_text()).group(1))


def test_finger_tendon_ratios_match_urdf():
    assert prop("MCP_MAX") == shaping.MCP_MAX
    assert prop("K_PIP") == shaping.K_PIP
    assert prop("K_DIP") == shaping.K_DIP


def test_thumb_maxima_match_urdf():
    urdf = {"thumb_swing": prop("SWING_MAX"), "thumb_opp": prop("OPP_MAX"),
            "thumb_mcp": prop("TMCP_MAX"), "thumb_ip": prop("TIP_MAX")}
    assert urdf == shaping.THUMB_MAX


def test_wrist_limits_match_urdf():
    np.testing.assert_allclose(shaping.WRIST_LIMITS, [[-prop("WRIST_PRON_MAX"), prop("WRIST_PRON_MAX")],
                                                      [-prop("WRIST_EXT_MAX"), prop("WRIST_FLEX_MAX")]])


def test_wrist_limits_match_the_tracker():
    import sys
    sys.path.insert(0, str(XACRO.parents[2] / "hand_tracker"))
    from hand_tracker.core.wrist import FLEXION_LIMITS, PRONATION_LIMITS
    np.testing.assert_allclose(shaping.WRIST_LIMITS, [PRONATION_LIMITS, FLEXION_LIMITS])


def test_finger_layout():
    out = shaping.joint_targets([1, 0, 0, 0, 0])
    np.testing.assert_allclose(out[:3], [1.57, 1.57 * 1.2, 1.57 * 0.7])
    assert np.all(out[3:] == 0)


def test_controllers_yaml_lists_the_same_joints():
    yaml = XACRO.parents[2] / "hand_bringup/config/controllers.yaml"
    names = re.search(r"joints: \[(.*?)\]", yaml.read_text(), re.S).group(1)
    assert [n.strip() for n in names.split(",")] == shaping.JOINT_NAMES

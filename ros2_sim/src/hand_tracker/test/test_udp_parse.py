import json
import sys
import types

import pytest

# udp_bridge imports rclpy/ROS messages; stub them so the parser can be tested without ROS.
for name in ("rclpy", "rclpy.node", "hand_interfaces", "hand_interfaces.msg"):
    sys.modules.setdefault(name, types.ModuleType(name))
sys.modules["rclpy.node"].Node = object
sys.modules["hand_interfaces.msg"].HandState = object

from hand_tracker.udp_bridge import parse_packet  # noqa: E402


def test_valid_packet():
    raw = json.dumps({"hand": "Right", "gesture": "Fist", "curl": [1, 0.5, 0, 0, 1]})
    assert parse_packet(raw) == ("Right", "Fist", [1.0, 0.5, 0.0, 0.0, 1.0], [0.0, 0.0])


def test_wrist_is_optional_and_validated():
    base = {"hand": "Right", "curl": [0, 0, 0, 0, 0]}
    assert parse_packet(json.dumps({**base, "wrist": [0.4, -0.2]}))[3] == pytest.approx([0.4, -0.2])
    assert parse_packet(json.dumps({**base, "wrist": [0.4]}))[3] == [0.0, 0.0]
    assert parse_packet(json.dumps({**base, "wrist": [float("nan"), 0]}))[3] == [0.0, 0.0]


def test_values_are_clamped():
    assert parse_packet(json.dumps({"hand": "Left", "curl": [2, -1, 0, 0, 0]}))[2][:2] == [1.0, 0.0]


@pytest.mark.parametrize("raw", [b"garbage", json.dumps({"hand": None}), json.dumps({"hand": "Right"}),
                                 json.dumps({"hand": "Right", "curl": [0, 0, 0]}),
                                 json.dumps({"hand": "Right", "curl": ["a"] * 5})])
def test_bad_packets_are_ignored(raw):
    assert parse_packet(raw) is None

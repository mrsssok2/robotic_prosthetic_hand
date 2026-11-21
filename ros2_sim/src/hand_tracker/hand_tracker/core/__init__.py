"""ROS-free tracking math (numpy only) so it can be unit-tested without a ROS install."""
from .landmarks import FINGER_ORDER, NUM_LANDMARKS
from .curl import CurlConfig, finger_curls, finger_curls_2d, has_depth, joint_flexions, thumb_sweep
from .gesture import GESTURES, classify_gesture, is_pinch

__all__ = [
    "FINGER_ORDER", "NUM_LANDMARKS", "CurlConfig", "finger_curls", "finger_curls_2d", "has_depth",
    "joint_flexions", "thumb_sweep", "GESTURES", "classify_gesture", "is_pinch",
]

"""Interchangeable hand trackers: ``make_tracker("yolo"|"mediapipe"|"classic")``.

Every tracker maps a BGR frame to a ``HandResult`` (or None when no hand is found):
curl per finger in FINGER_ORDER plus, where the method has them, 21 image-space keypoints.
"""
from .base import HandResult, Tracker, curl_from_image_keypoints


def make_tracker(name: str, **kw) -> Tracker:
    if name == "mediapipe":
        from .mediapipe_tracker import MediaPipeTracker
        return MediaPipeTracker(**kw)
    if name == "yolo":
        from .yolo_tracker import YoloTracker
        return YoloTracker(**kw)
    if name == "classic":
        from .classic_tracker import ClassicTracker
        return ClassicTracker(**kw)
    raise ValueError(f"unknown tracker {name!r} (yolo | mediapipe | classic)")


__all__ = ["HandResult", "Tracker", "make_tracker", "curl_from_image_keypoints"]

"""Baseline: MediaPipe Hands (Python, CPU)."""
import cv2
import numpy as np

from ..core import finger_curls, wrist_angles
from .base import HandResult, Tracker, curl_from_image_keypoints


class MediaPipeTracker(Tracker):
    name = "mediapipe"

    def __init__(self, static_image_mode=False, use_world=True, mirrored=True, **_):
        import mediapipe as mp
        self.use_world = use_world
        self.mirrored = mirrored      # the frames are flipped for a selfie view (hand_cam.py does that for the webcam)
        self.hands = mp.solutions.hands.Hands(static_image_mode=static_image_mode, max_num_hands=1,
                                              min_detection_confidence=0.5, min_tracking_confidence=0.5)

    def process(self, bgr):
        res = self.hands.process(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        if not res.multi_hand_landmarks:
            return None
        h, w = bgr.shape[:2]
        lm = res.multi_hand_landmarks[0].landmark
        kp = np.array([[p.x * w, p.y * h] for p in lm])
        curl_2d = curl_from_image_keypoints(kp)
        curl, extra = curl_2d, {"curl_2d": curl_2d}
        label = res.multi_handedness[0].classification[0].label
        if self.use_world and res.multi_hand_world_landmarks:
            world = np.array([[p.x, p.y, p.z] for p in res.multi_hand_world_landmarks[0].landmark])
            curl = finger_curls(world)
            extra["wrist"] = wrist_angles(world, label, self.mirrored)   # (pronation, flexion) rad, palm normal + wrist->middle-MCP
        score = res.multi_handedness[0].classification[0].score
        return HandResult(curl, kp, float(score), extra)

    def close(self):
        self.hands.close()

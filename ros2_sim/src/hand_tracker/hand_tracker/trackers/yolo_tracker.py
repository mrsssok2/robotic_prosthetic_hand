"""Own detector: YOLO11 pose model (21 hand keypoints) exported to ONNX, run with onnxruntime.

Needs only numpy, opencv and onnxruntime (no torch / ultralytics at run time), so it works in the
Windows venv. Letterbox -> model -> decode best box + 21 keypoints -> curl from the keypoints.
"""
import pathlib

import cv2
import numpy as np

from .base import HandResult, Tracker, curl_from_image_keypoints

DEFAULT_MODEL = pathlib.Path(__file__).resolve().parents[3] / "models" / "hand_pose_n640.onnx"


def letterbox(img, size):
    h, w = img.shape[:2]
    r = size / max(h, w)
    nh, nw = round(h * r), round(w * r)
    out = np.full((size, size, 3), 114, np.uint8)
    top, left = (size - nh) // 2, (size - nw) // 2
    out[top:top + nh, left:left + nw] = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    return out, r, left, top


class YoloTracker(Tracker):
    name = "yolo"

    def __init__(self, model=None, conf=0.25, providers=None, **_):
        import onnxruntime as ort
        path = pathlib.Path(model) if model else DEFAULT_MODEL
        if not path.exists():
            raise FileNotFoundError(f"{path} not found - run tracking/train_yolo.py (or copy models/hand_pose_n640.onnx)")
        avail = ort.get_available_providers()
        providers = providers or [p for p in ("CUDAExecutionProvider", "CPUExecutionProvider") if p in avail]
        self.sess = ort.InferenceSession(str(path), providers=providers)
        inp = self.sess.get_inputs()[0]
        self.input_name, self.size = inp.name, int(inp.shape[2])
        self.conf = conf

    def process(self, bgr):
        img, r, left, top = letterbox(bgr, self.size)
        x = np.ascontiguousarray(img[:, :, ::-1].transpose(2, 0, 1), dtype=np.float32)[None] / 255.0
        out = self.sess.run(None, {self.input_name: x})[0][0]   # (4 + 1 + 63, N)
        scores = out[4]
        i = int(np.argmax(scores))
        if scores[i] < self.conf:
            return None
        kp = out[5:, i].reshape(21, 3)
        xy = np.stack([(kp[:, 0] - left) / r, (kp[:, 1] - top) / r], axis=1)
        return HandResult(curl_from_image_keypoints(xy), xy, float(scores[i]))

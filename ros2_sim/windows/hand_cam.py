"""Windows-side hand tracker: webcam -> MediaPipe Hands -> 5 finger curls + gesture -> UDP to ROS.

Same idea as the original gesture_control.py (OpenCV preview, MediaPipe Hands, one hand,
confidence 0.5) but the finger values are proportional (0..1) and sent to the ROS bridge
node in Docker, which drives the simulated hand.

    python hand_cam.py                       # webcam 0, MediaPipe, UDP -> 127.0.0.1:9870
    python hand_cam.py --tracker yolo        # own YOLO11 pose model (models/hand_pose_n640.onnx, onnxruntime)
    python hand_cam.py --tracker classic     # skin segmentation + contour, no learning
    python hand_cam.py --camera 1            # another camera
    python hand_cam.py --source clip.mp4     # video file (or a still image) instead of the webcam

Press q (in the preview window) to quit.

UDP packet (JSON, one per frame):
    {"t": <epoch s>, "hand": "Right", "gesture": "Fist", "tracker": "yolo", "track_ms": 21.3,
     "curl": [little, index, middle, ring, thumb]}
    {"t": <epoch s>, "hand": null}                      # no hand visible
"""
import argparse
import json
import pathlib
import socket
import sys
import time

import cv2

# The tested, ROS-free tracking math lives in the repo: src/hand_tracker/hand_tracker/core
_here = pathlib.Path(__file__).resolve().parent
for candidate in (_here.parent / "src" / "hand_tracker", _here / "src" / "hand_tracker", _here):
    if (candidate / "hand_tracker" / "core").is_dir():
        sys.path.insert(0, str(candidate))
        break
from hand_tracker.core import FINGER_ORDER, classify_gesture, is_pinch  # noqa: E402
from hand_tracker.trackers import make_tracker  # noqa: E402

NAMES = [n.capitalize() for n in FINGER_ORDER]  # Little, Index, Middle, Ring, Thumb


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1", help="ROS bridge address (Docker publishes it on localhost)")
    ap.add_argument("--port", type=int, default=9870)
    ap.add_argument("--tracker", choices=("yolo", "mediapipe", "classic"), default="mediapipe",
                    help="yolo = own YOLO11 pose model (ONNX), mediapipe = baseline, classic = skin + contour, no learning")
    ap.add_argument("--model", help="ONNX file for --tracker yolo (default: models/hand_pose_n640.onnx)")
    ap.add_argument("--device", choices=("auto", "cuda", "dml", "cpu"), default="auto",
                    help="--tracker yolo: auto = CUDA > DirectML (Windows GPU) > CPU; falls back to CPU if the GPU path fails")
    ap.add_argument("--yolo-conf", type=float, default=0.25, help="detection confidence for --tracker yolo")
    ap.add_argument("--camera", type=int, default=0, help="webcam index")
    ap.add_argument("--source", help="video file or image instead of the webcam")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--no-window", action="store_true", help="no preview (headless tests)")
    ap.add_argument("--max-frames", type=int, default=0, help="stop after N frames (tests)")
    return ap.parse_args()


def open_source(args):
    if args.source:
        img = cv2.imread(args.source) if args.source.lower().endswith((".jpg", ".jpeg", ".png", ".bmp")) else None
        if img is not None:
            return None, img
        cap = cv2.VideoCapture(args.source)
    else:
        cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW) if sys.platform == "win32" else cv2.VideoCapture(args.camera)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        sys.exit("cannot open the camera/video (try --camera 1, and close other apps using the webcam)")
    return cap, None


def draw_overlay(frame, fps, gesture, curl):
    cv2.putText(frame, f"FPS: {fps:4.1f}", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(frame, f"Gesture: {gesture}", (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
    if curl is None:
        return
    for i, (name, c) in enumerate(zip(NAMES, curl)):
        y = 95 + i * 26
        cv2.putText(frame, f"{name:<6} {c:4.2f}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.rectangle(frame, (140, y - 14), (140 + 100, y + 2), (80, 80, 80), 1)
        cv2.rectangle(frame, (140, y - 14), (140 + int(100 * c), y + 2), (0, 255, 0), -1)


def main():
    args = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.host, args.port)
    cap, still = open_source(args)
    tracker = make_tracker(args.tracker, **({"model": args.model, "conf": args.yolo_conf, "device": args.device} if args.tracker == "yolo" else {}))
    fps, last, frames = 0.0, time.time(), 0
    if args.tracker == "yolo":
        print(f"yolo runs on: {tracker.active_provider}")
    print(f"tracker: {args.tracker} - sending to udp://{args.host}:{args.port} - press q in the preview to quit")
    while True:
        if cap is None:
            frame = still.copy()
        else:
            ok, frame = cap.read()
            if not ok:
                break
        if not args.source:
            frame = cv2.flip(frame, 1)  # mirrored selfie view, like the original
        t0 = time.perf_counter()
        result = tracker.process(frame)
        track_ms = (time.perf_counter() - t0) * 1000

        packet, curl, gesture = {"t": time.time(), "hand": None}, None, "No hand"
        if result is not None:
            curl = result.curl
            gesture = classify_gesture(curl)
            if result.keypoints is not None and is_pinch(result.keypoints, curl):
                gesture = "Pinch"
            packet = {"t": time.time(), "hand": "Right", "gesture": gesture, "tracker": args.tracker,
                      "track_ms": round(track_ms, 2), "curl": [round(float(c), 4) for c in curl]}
            if result.keypoints is not None:
                for x, y in result.keypoints:
                    cv2.circle(frame, (int(x), int(y)), 3, (0, 255, 255), -1)
        sock.sendto(json.dumps(packet).encode(), dest)

        frames += 1
        now = time.time()
        if now - last >= 0.5:
            fps, frames, last = frames / (now - last), 0, now
        if not args.no_window:
            draw_overlay(frame, fps, gesture, curl)
            cv2.putText(frame, f"{args.tracker} {track_ms:4.1f} ms", (10, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.imshow("Hand tracker (q to quit)", frame)
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break
        if args.max_frames:
            args.max_frames -= 1
            if args.max_frames == 0:
                break
        if cap is None:
            time.sleep(0.03)
    if cap is not None:
        cap.release()
    tracker.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

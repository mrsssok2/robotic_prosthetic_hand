"""Train a small YOLO11 pose model on the Ultralytics hand-keypoints dataset (21 keypoints) and export ONNX.

Runs in the gurdy2/vision:cuda image (torch + ultralytics + RTX GPU); see tracking/run_in_docker.sh.

    python tracking/train_yolo.py --model yolo11n-pose.pt --epochs 60 --imgsz 320 --name n320

Dataset labels were produced by running MediaPipe Hands on the images (Ultralytics docs), so the model
learns to imitate MediaPipe; it is NOT scored against independent human annotation.
"""
import argparse
import pathlib
import shutil

from ultralytics import YOLO

ROOT = pathlib.Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="yolo11n-pose.pt")
    ap.add_argument("--data", default=str(ROOT / "data" / "hand-keypoints.yaml"))
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--imgsz", type=int, default=320)
    ap.add_argument("--batch", type=int, default=64)
    ap.add_argument("--name", default="n320")
    ap.add_argument("--workers", type=int, default=4)
    a = ap.parse_args()
    model = YOLO(a.model)
    model.train(data=a.data, epochs=a.epochs, imgsz=a.imgsz, batch=a.batch, workers=a.workers,
                project=str(ROOT / "runs"), name=a.name, exist_ok=True, patience=20, seed=0,
                fliplr=0.5, degrees=15, amp=True, plots=True)
    best = ROOT / "runs" / a.name / "weights" / "best.pt"
    onnx = YOLO(str(best)).export(format="onnx", imgsz=a.imgsz, opset=17, simplify=True, dynamic=False)
    (ROOT / "models").mkdir(exist_ok=True)
    shutil.copy(onnx, ROOT / "models" / f"hand_pose_{a.name}.onnx")
    print("exported", ROOT / "models" / f"hand_pose_{a.name}.onnx")


if __name__ == "__main__":
    main()

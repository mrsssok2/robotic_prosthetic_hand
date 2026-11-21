"""Export YOLO pose weights to ONNX:  python tracking/export_onnx.py runs/n320/weights/best.pt models/hand_pose_n320.onnx"""
import pathlib
import shutil
import sys

from ultralytics import YOLO

weights, dest = sys.argv[1], pathlib.Path(sys.argv[2])
imgsz = int(sys.argv[3]) if len(sys.argv) > 3 else 320
out = YOLO(weights).export(format="onnx", imgsz=imgsz, opset=17, simplify=True, dynamic=False)
dest.parent.mkdir(parents=True, exist_ok=True)
shutil.copy(out, dest)
print("wrote", dest)

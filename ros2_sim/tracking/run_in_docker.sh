#!/bin/bash
# Run a tracking/ script in the CUDA vision image, with MediaPipe 0.10.14 available from data/pylibs
# (installed once:  pip install --target data/pylibs mediapipe==0.10.14 --no-deps ; pip install --target data/pylibs protobuf==4.25.3).
# The repo is mounted at its host path so file lists with absolute paths work inside and outside.
#   tracking/run_in_docker.sh python tracking/train_yolo.py --epochs 60
cd "$(dirname "$0")/.."
exec docker run --rm --gpus all --ipc=host -v "$PWD":"$PWD" -w "$PWD" \
  -e PYTHONPATH="$PWD/data/pylibs:$PWD/src/hand_tracker" -e YOLO_CONFIG_DIR="$PWD/data/yolo_cfg" \
  gurdy2/vision:cuda "$@"

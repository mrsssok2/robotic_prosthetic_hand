#!/bin/bash
# One command: builds the workspace in Docker and opens Gazebo. Then start windows\hand_cam.py on Windows.
# Extra args go to the launch file:  ./run.sh input:=fake   (no camera)   ./run.sh input:=browser   gui:=false
cd "$(dirname "$0")"
echo "Waiting for hand_cam.py on Windows (UDP localhost:9870). See windows/README.md."
exec docker compose run --rm --service-ports hand /home/ubuntu/hand_ros2/scripts/start.sh "$@"

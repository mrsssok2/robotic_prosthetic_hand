#!/bin/bash
# Host helper: run a command inside the running `${SIM:-handsim}` container (see sim_up.sh) with ROS + Gazebo set up
# (goes through the image entrypoint, which sets GZ_PARTITION; plain `docker exec` would not, and then `gz service` calls
# silently reach nobody).   scripts/sim_exec.sh python3 scripts/grasp_test.py --trials 5
cd "$(dirname "$0")/.."
exec docker exec ${SIM:-handsim} /hand_entrypoint.sh bash -c 'cd ~/hand_ros2 && exec "$@"' _ "$@"

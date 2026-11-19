#!/bin/bash
# Runs INSIDE the container: build the workspace, then launch. Extra args go to the launch file.
set -e
cd ~/hand_ros2
colcon build --symlink-install --event-handlers console_cohesion- 2>&1 | tail -n 15
set +u; source install/setup.bash
exec ros2 launch hand_bringup hand.launch.py "$@"

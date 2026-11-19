#!/bin/bash
# Source ROS 2 and the workspace overlay (if built), then run the command.
export GZ_PARTITION="${GZ_PARTITION:-$(hostname)}"
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/hand_ros2/install/setup.bash" ] && source "$HOME/hand_ros2/install/setup.bash"
exec "$@"

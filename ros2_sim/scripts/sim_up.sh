#!/bin/bash
# Host helper: start the headless simulation in a background container named `${SIM:-handsim}`
# (extra args go to the launch file, e.g. ./scripts/sim_up.sh grasp:=true), wait until the controllers are up.
cd "$(dirname "$0")/.."
docker rm -f ${SIM:-handsim} >/dev/null 2>&1
docker run -d --name ${SIM:-handsim} --init -v "$PWD":/home/ubuntu/hand_ros2 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST --memory 6g \
  hand_ros2:jazzy bash -c "cd ~/hand_ros2 && ${SKIP_BUILD:+true || }colcon build --symlink-install 2>&1 | tail -3; source install/setup.bash; ros2 launch hand_bringup hand.launch.py input:=udp gui:=false $*" >/dev/null
for _ in $(seq 120); do
  docker logs ${SIM:-handsim} 2>&1 | grep -q "Configured and activated finger_position_controller" && { sleep 3; echo "sim up"; exit 0; }
  sleep 2
done
echo "sim did not come up"; docker logs ${SIM:-handsim} 2>&1 | tail -20; exit 1

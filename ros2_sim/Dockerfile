# Built on the existing gurdy2/ros:jazzy image (ROS 2 Jazzy, Gazebo Harmonic, ros2_control, RViz,
# python3-websockets/numpy) -- nothing extra to download. Runs as user `ubuntu` (UID 1000).
FROM gurdy2/ros:jazzy
COPY --chmod=755 docker/entrypoint.sh /hand_entrypoint.sh
WORKDIR /home/ubuntu/hand_ros2
ENTRYPOINT ["/hand_entrypoint.sh"]
CMD ["bash"]

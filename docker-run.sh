#!/bin/bash
docker run -it --rm \
  --privileged \
  --net=host \
  -v /dev:/dev \
  -v /run/udev:/run/udev \
  --group-add video \
  -e DISPLAY=$DISPLAY \
  -e ROS_DOMAIN_ID=10 \
  -e HOME=/tmp \
  camera_ros_humble \
  bash -c "source /opt/ros/humble/setup.bash && source /app/install/setup.bash && ros2 run camera_ros camera_node"


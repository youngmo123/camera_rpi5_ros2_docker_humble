#!/bin/bash

docker run -it --rm \
  --privileged \
  --net=host \
  -v /dev:/dev \
  -v /run/udev:/run/udev \
  --group-add video \
  -e ROS_DOMAIN_ID=10 \
  -e HOME=/tmp \
  camera_ros_humble \
  ros2 run camera_ros camera_node

#!/bin/bash

WORKSPACE="/home/teame/husky/autoware"
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
# Output bag name with timestamp
BAG_NAME="sensor_bag_$(date +%Y%m%d_%H%M%S)"

echo "Recording ROS 2 bag: ${BAG_NAME}"
echo "Press Ctrl+C to stop recording."

ros2 bag record \
  -o "${BAG_NAME}" \
  /velodyne_points \
  /fixposition/fpa/rawimu \
  /fixposition/fpa/corrimu \
  /fixposition/gnss1 \
  /fixposition/gnss2 \
  /fixposition/fpa/llh \
  /fixposition/fpa/odometry \
  /fixposition/odometry_enu \
  /fixposition/poiimu \
  /tf \
  /tf_static \
  /husky/platform/odom \
  /husky/platform/odom/filtered


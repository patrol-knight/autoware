#!/usr/bin/env bash

WORKSPACE="/home/teame/husky/autoware"
source "${WORKSPACE}/husky_control.conf"
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
# Output bag name with timestamp
BAG_NAME="autoware_bag_$(date +%Y%m%d_%H%M%S)"

echo "Recording ROS 2 bag: ${BAG_NAME}"
echo "Press Ctrl+C to stop recording."
ros2 bag record \
  -o "${BAG_NAME}" \
  /velodyne_points \
  /sensing/lidar/obstacle_stop \
  /sensing/lidar/obstacle_count \
  /sensing/lidar/obstacle_exe_time_ms \
  /fixposition/fpa/corrimu \
  /fixposition/gnss1 \
  "/husky/platform/odom" \
  "/husky/platform/odom/filtered" \
  "/husky/joy_teleop/joy" \
  "/husky/joy_teleop/cmd_vel" \
  "/husky/rc_teleop/cmd_vel" \
  "/husky/twist_marker_server/cmd_vel" \
  "$HUSKY_CMD_VEL_TOPIC" \
  "/husky/platform/cmd_vel_unstamped" \
  "/husky/platform/emergency_stop" \
  /sensing/imu/imu_data \
  /sensing/vehicle_velocity_converter/twist_with_covariance \
  /vehicle/status/velocity_status \
  /vehicle/status/control_mode \
  /localization/pose_estimator/pose \
  /localization/pose_estimator/pose_with_covariance \
  /localization/pose_estimator/exe_time_ms \
  /localization/pose_estimator/nearest_voxel_transformation_likelihood \
  /localization/pose_twist_fusion_filter/pose \
  /localization/pose_with_covariance \
  /localization/kinematic_state \
  /localization/initialization_state \
  /localization/twist_estimator/twist_with_covariance \
  /planning/mission_planning/goal \
  /planning/mission_planning/route \
  /planning/route_state \
  /planning/scenario_planning/velocity_smoother/trajectory \
  /planning/trajectory \
  /control/husky/cmd_vel \
  /control/husky/tracking_state \
  /control/husky/bridge_state \
  /control/trajectory_follower/lateral/predicted_trajectory \
  /system/operation_mode/state \
  /tf \
  /tf_static \
  /diagnostics \
  /cpu_monitor/cpu_usage \
  /mem_monitor/memory_status \
  /robot_status

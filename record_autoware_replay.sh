#!/usr/bin/env bash
set -euo pipefail

if [[ $# -gt 1 ]]; then
  echo "Usage: $0 [OUTPUT_BAG_PATH]" >&2
  exit 2
fi

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_BAG_PATH="${1:-autoware_replay_bag_$(date +%Y%m%d_%H%M%S)}"
source "$WORKSPACE/husky_control.conf"

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u

echo "Recording replay inputs and pipeline outputs to $OUTPUT_BAG_PATH"
echo "Replay sensor inputs with replay_husky_sensing_bag.sh; do not replay the whole bag into a running Autoware stack."
# Keep the original LiDAR cloud for sensing replay. Processed clouds are
# regenerated and would duplicate most of the bag's storage cost.
exec ros2 bag record -o "$OUTPUT_BAG_PATH" \
  /velodyne_points \
  /fixposition/fpa/corrimu \
  "$HUSKY_ROBOT_NAMESPACE/platform/odom" \
  "$HUSKY_ROBOT_NAMESPACE/platform/odom/filtered" \
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
  /localization/twist_estimator/twist_with_covariance \
  /planning/mission_planning/goal \
  /planning/mission_planning/route \
  /planning/scenario_planning/velocity_smoother/trajectory \
  /planning/trajectory \
  /control/husky/cmd_vel \
  /control/husky/tracking_state \
  /control/husky/bridge_state \
  /control/trajectory_follower/lateral/predicted_trajectory \
  /system/operation_mode/state \
  /tf \
  /tf_static \
  /diagnostics

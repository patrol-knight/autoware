#!/usr/bin/env bash
set -euo pipefail

if [[ $# -gt 1 ]]; then
  echo "Usage: $0 [OUTPUT_BAG_PATH]" >&2
  exit 2
fi

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_BAG_PATH="${1:-control_bag_$(date +%Y%m%d_%H%M%S)}"
source "$WORKSPACE/husky_control.conf"

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u

echo "Recording Husky control topics to $OUTPUT_BAG_PATH"
echo "Platform command topic: $HUSKY_CMD_VEL_TOPIC"
echo "Press Ctrl+C to finish the bag."

# Keep the planner input, controller output, actual platform command, and both
# localization and wheel-odometry feedback for latency and tracking analysis.
# Topics that are not published yet (such as the vehicle bridge) may be absent.
exec ros2 bag record -o "$OUTPUT_BAG_PATH" \
  /planning/trajectory \
  /planning/scenario_planning/velocity_smoother/trajectory \
  /localization/kinematic_state \
  /control/husky/cmd_vel \
  /control/husky/tracking_state \
  /control/husky/bridge_state \
  /control/trajectory_follower/lateral/predicted_trajectory \
  /control/command/control_cmd \
  "$HUSKY_CMD_VEL_TOPIC" \
  "$HUSKY_ROBOT_NAMESPACE/platform/cmd_vel_unstamped" \
  "$HUSKY_ROBOT_NAMESPACE/joy_teleop/cmd_vel" \
  "$HUSKY_ROBOT_NAMESPACE/rc_teleop/cmd_vel" \
  "$HUSKY_ROBOT_NAMESPACE/twist_marker_server/cmd_vel" \
  "$HUSKY_ROBOT_NAMESPACE/platform/emergency_stop" \
  "$HUSKY_ROBOT_NAMESPACE/platform/odom" \
  "$HUSKY_ROBOT_NAMESPACE/platform/odom/filtered" \
  /vehicle/status/control_mode \
  /vehicle/status/velocity_status \
  /tf \
  /tf_static \
  /diagnostics \
  /clock

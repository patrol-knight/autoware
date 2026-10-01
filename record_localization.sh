#!/usr/bin/env bash
set -euo pipefail

if [[ $# -gt 1 ]]; then
  echo "Usage: $0 [OUTPUT_BAG_PATH]" >&2
  exit 2
fi

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_BAG_PATH="${1:-localization_bag_$(date +%Y%m%d_%H%M%S)}"

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u

echo "Recording localization topics to $OUTPUT_BAG_PATH"
exec ros2 bag record -o "$OUTPUT_BAG_PATH" \
  /localization/pose_estimator/pose \
  /localization/pose_estimator/pose_with_covariance \
  /localization/pose_twist_fusion_filter/pose \
  /localization/pose_with_covariance \
  /localization/twist_estimator/twist_with_covariance \
  /diagnostics \
  /clock

#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 BAG_PATH [ros2 bag play options]" >&2
  exit 2
fi

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_PATH="$1"
shift
set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u

# The Husky URDF owns sensor static TF. Replay only the recorded driver inputs.
exec ros2 bag play "$BAG_PATH" --clock "$@" --topics \
  /velodyne_points \
  /fixposition/fpa/corrimu \
  /fixposition/gnss1 \
  /husky/platform/odom/filtered

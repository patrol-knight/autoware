#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAP_PATH="${MAP_PATH:-/home/teame/husky/map}"
VEHICLE_MODEL="${VEHICLE_MODEL:-sample_vehicle}"
SENSOR_MODEL="${SENSOR_MODEL:-sample_sensor_kit}"
POINTCLOUD_MAP_FILE="${POINTCLOUD_MAP_FILE:-pointcloud_map.pcd}"

for file in \
  /opt/ros/humble/setup.bash \
  "$WORKSPACE/install/setup.bash" \
  "$MAP_PATH/lanelet2_map.osm" \
  "$MAP_PATH/map_projector_info.yaml" \
  "$MAP_PATH/$POINTCLOUD_MAP_FILE"; do
  if [[ ! -f "$file" ]]; then
    echo "Missing required file: $file" >&2
    exit 1
  fi
done

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

echo "Starting Autoware with map: $MAP_PATH"
echo "Vehicle interface is disabled; sample models need Husky-specific integration."

exec ros2 launch autoware_launch autoware.launch.xml \
  map_path:="$MAP_PATH" \
  vehicle_model:="$VEHICLE_MODEL" \
  sensor_model:="$SENSOR_MODEL" \
  system_run_mode:=online \
  launch_sensing_driver:=false \
  launch_vehicle_interface:=false \
  launch_control:=true \
  use_sim_time:=false \
  pointcloud_map_file:="$POINTCLOUD_MAP_FILE"

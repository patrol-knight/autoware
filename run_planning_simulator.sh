#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAP_PATH="${MAP_PATH:-$WORKSPACE/../map/demo}"
VEHICLE_MODEL="${VEHICLE_MODEL:-sample_vehicle}"
SENSOR_MODEL="${SENSOR_MODEL:-sample_sensor_kit}"

for file in \
  /opt/ros/humble/setup.bash \
  "$WORKSPACE/install/setup.bash" \
  "$MAP_PATH/lanelet2_map.osm" \
  "$MAP_PATH/pointcloud_map.pcd" \
  "$MAP_PATH/map_projector_info.yaml"; do
  if [[ ! -f "$file" ]]; then
    echo "Missing required file: $file" >&2
    exit 1
  fi
done

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u

echo "Starting planning simulator with map: $MAP_PATH"
exec ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:="$MAP_PATH" \
  vehicle_model:="$VEHICLE_MODEL" \
  sensor_model:="$SENSOR_MODEL" \
  rviz:=true \
  rviz_config_name:=husky_planning.rviz \
  "$@"

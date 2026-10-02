#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAP_PATH="${MAP_PATH:-/home/teame/husky/map}"
# Autoware loads vehicle and sensor configuration for both sensing and localization.
VEHICLE_MODEL="${VEHICLE_MODEL:-husky_vehicle}"
SENSOR_MODEL="${SENSOR_MODEL:-husky_sensor_kit}"

for file in \
  /opt/ros/humble/setup.bash \
  "$WORKSPACE/install/setup.bash"; do
  if [[ ! -f "$file" ]]; then
    echo "Missing required file: $file" >&2
    exit 1
  fi
done

if [[ "$VEHICLE_MODEL" == husky_vehicle ]]; then
  vehicle_params="$WORKSPACE/install/husky_vehicle_description/share/husky_vehicle_description/config/vehicle_info.param.yaml"
  if [[ ! -f "$vehicle_params" ]]; then
    echo "Missing Husky vehicle dimensions: $vehicle_params" >&2
    echo "Start from husky_vehicle_description/config/vehicle_info.param.yaml.example" >&2
    exit 1
  fi
  if [[ $(<"$vehicle_params") == *REPLACE_ME* ]]; then
    echo "Husky vehicle dimensions still contain REPLACE_ME: $vehicle_params" >&2
    exit 1
  fi
fi

if [[ "$SENSOR_MODEL" == husky_sensor_kit ]]; then
  sensor_calibration="$WORKSPACE/install/husky_sensor_kit_description/share/husky_sensor_kit_description/config/sensors_calibration.yaml"
  if [[ ! -f "$sensor_calibration" ]]; then
    echo "Missing Husky sensor calibration: $sensor_calibration" >&2
    echo "Check husky_sensor_kit_description/config/sensors_calibration.yaml" >&2
    exit 1
  fi
  if [[ $(<"$sensor_calibration") == *REPLACE_ME* ]]; then
    echo "Husky sensor calibration still contains REPLACE_ME: $sensor_calibration" >&2
    exit 1
  fi
fi

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u

echo "Starting autoware.launch.xml with Husky vehicle and sensor models."
echo "Vehicle model: $VEHICLE_MODEL; sensor model: $SENSOR_MODEL."

exec ros2 launch autoware_launch autoware.launch.xml \
  map_path:="$MAP_PATH" \
  vehicle_model:="$VEHICLE_MODEL" \
  sensor_model:="$SENSOR_MODEL" \
  system_run_mode:=logging_simulation \
  launch_vehicle:=true \
  launch_vehicle_interface:=false \
  launch_system:=false \
  launch_map:=true \
  launch_sensing:=true \
  launch_sensing_driver:=false \
  launch_localization:=true \
  localization_gnss_enabled:=false \
  launch_perception:=false \
  launch_planning:=true \
  planning_module_preset:=husky_no_perception \
  launch_planning_test_adaptors:=true \
  launch_control:=false \
  launch_api:=false \
  launch_system_monitor:=false \
  launch_dummy_diag_publisher:=false \
  launch_pointcloud_container:=true \
  rviz:=true \
  rviz_config_name:=husky_planning.rviz \
  rviz_respawn:=false \
  use_sim_time:=true \
  "$@"

#!/usr/bin/env bash

set -e

WORKSPACE="/home/teame/husky/autoware"
VLP16_CONFIG="/home/teame/husky/config/VLP16.param.yaml"

# Check required files
if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
    echo "Error: ${WORKSPACE}/install/setup.bash does not exist."
    echo "Please build the required ROS 2 packages first."
    exit 1
fi

if [[ ! -f "${VLP16_CONFIG}" ]]; then
    echo "Error: ${VLP16_CONFIG} does not exist."
    exit 1
fi

# Load ROS 2 and workspace environments
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"

PIDS=()

cleanup()
{
    echo
    echo "Stopping sensor drivers..."

    if [[ ${#PIDS[@]} -gt 0 ]]; then
        kill -SIGINT "${PIDS[@]}" 2>/dev/null || true
        wait "${PIDS[@]}" 2>/dev/null || true
    fi

    echo "All sensor drivers stopped."
}

trap cleanup SIGINT SIGTERM EXIT

echo "Starting VLP-16 driver..."
ros2 launch nebula_velodyne velodyne_launch_all_hw.xml \
    sensor_model:=VLP16 \
    config_file:="${VLP16_CONFIG}" &
PIDS+=($!)

echo "Starting RealSense driver..."
ros2 launch realsense2_camera rs_launch.py &
PIDS+=($!)

echo "Starting Fixposition driver..."
ros2 launch fixposition_driver_ros2 node.launch &
PIDS+=($!)

echo
echo "All sensor drivers started."

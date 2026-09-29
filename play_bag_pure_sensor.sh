#!/bin/bash

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

ros2 bag play $1 --clock 200 --topics \
    /velodyne_points \
    /fixposition/fpa/corrimu \
    /husky/platform/odom/filtered \
    /fixposition/gnss1
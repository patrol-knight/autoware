"""Publish the measured Husky sensor frames when vehicle launch is disabled."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    vehicle_xacro = Path(get_package_share_directory("tier4_vehicle_launch")) / "urdf" / "vehicle.xacro"
    robot_description = xacro.process_file(
        str(vehicle_xacro),
        mappings={"vehicle_model": "husky_vehicle", "sensor_model": "husky_sensor_kit"},
    ).toxml()

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="husky_sensor_robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            )
        ]
    )

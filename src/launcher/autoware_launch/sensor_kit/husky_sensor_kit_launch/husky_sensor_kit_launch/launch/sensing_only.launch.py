"""Compatibility wrapper for starting the Husky vehicle model and sensing launch."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sensing_launch = Path(get_package_share_directory("husky_sensor_kit_launch")) / "launch" / "sensing.launch.xml"
    vehicle_launch = Path(get_package_share_directory("tier4_vehicle_launch")) / "launch" / "vehicle.launch.xml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_driver",
                default_value="true",
                description="Start the VLP-16 and Fixposition hardware drivers",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(str(vehicle_launch)),
                launch_arguments={
                    "vehicle_model": "husky_vehicle",
                    "sensor_model": "husky_sensor_kit",
                    "launch_vehicle_interface": "false",
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(str(sensing_launch)),
                launch_arguments={
                    "launch_driver": LaunchConfiguration("launch_driver"),
                    "launch_gnss_poser": "false",
                }.items(),
            ),
        ]
    )

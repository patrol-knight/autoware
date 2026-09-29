"""Compatibility wrapper for starting the Husky sensing launch directly."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sensing_launch = Path(get_package_share_directory("husky_sensor_kit_launch")) / "launch" / "sensing.launch.xml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_driver",
                default_value="true",
                description="Start the VLP-16 and Fixposition hardware drivers",
            ),
            DeclareLaunchArgument("publish_sensor_tf", default_value="true"),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(str(sensing_launch)),
                launch_arguments={
                    "launch_driver": LaunchConfiguration("launch_driver"),
                    "launch_vehicle": "false",
                    "publish_sensor_tf": LaunchConfiguration("publish_sensor_tf"),
                    "launch_gnss_poser": "false",
                }.items(),
            ),
        ]
    )

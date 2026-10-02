"""Minimal interfaces for inspecting planning without Autoware control."""

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="autoware_launch",
                executable="planning_test_adaptor.py",
                name="planning_test_adaptor",
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                output="screen",
            )
        ]
    )

"""Run the Husky GNSS poser until localization initializes successfully."""

import signal

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.events.process import SignalProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gnss_poser = Node(
        package="autoware_gnss_poser",
        executable="gnss_poser",
        name="gnss_poser",
        output="both",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("husky_sensor_kit_launch"), "config", "gnss_poser.param.yaml"]
            )
        ],
        remappings=[
            ("fix", "/sensing/gnss/fix"),
            ("gnss_pose", "/sensing/gnss/pose"),
            ("gnss_pose_cov", "/sensing/gnss/pose_with_covariance"),
            ("gnss_fixed", "/sensing/gnss/fixed"),
        ],
    )
    monitor = Node(
        package="husky_sensor_kit_launch",
        executable="gnss_poser_initialization_monitor.py",
        name="gnss_poser_initialization_monitor",
        output="screen",
    )

    def stop_gnss_after_initialization(event, _context):
        if event.returncode != 0:
            return []
        return [
            EmitEvent(
                event=SignalProcess(
                    signal_number=signal.SIGINT,
                    process_matcher=matches_action(gnss_poser),
                )
            )
        ]

    return LaunchDescription(
        [
            RegisterEventHandler(
                OnProcessExit(target_action=monitor, on_exit=stop_gnss_after_initialization)
            ),
            gnss_poser,
            monitor,
        ]
    )

"""Process the single VLP-16 cloud with Autoware's LiDAR components."""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    components = [
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::DistortionCorrectorComponent",
            name="distortion_corrector_node",
            remappings=[
                ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
                ("~/input/imu", "/sensing/imu/imu_data"),
                ("~/input/pointcloud", "/velodyne_points"),
                ("~/output/pointcloud", "/sensing/lidar/concatenated/pointcloud"),
            ],
            parameters=[{
                "base_frame": "base_link",
                "use_imu": True,
                "use_3d_distortion_correction": False,
                "update_azimuth_and_distance": False,
                "processing_time_threshold_sec": 0.01,
                "timestamp_mismatch_fraction_threshold": 0.01,
            }],
        ),
    ]

    return LaunchDescription([
        ComposableNodeContainer(
            name="husky_lidar_preprocessor",
            namespace="/sensing/lidar/top",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=components,
            output="screen",
        )
    ])

"""Process the single VLP-16 cloud with Autoware's LiDAR components."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml


def generate_launch_description():
    vehicle_config = Path(get_package_share_directory("husky_vehicle_description")) / "config" / "vehicle_info.param.yaml"
    sensor_config = Path(get_package_share_directory("husky_sensor_kit_description")) / "config" / "sensors_calibration.yaml"
    vehicle = yaml.safe_load(vehicle_config.read_text())["/**"]["ros__parameters"]
    calibration = yaml.safe_load(sensor_config.read_text())

    # Clearpath's current base_link is at the bottom-plate center. The wheel
    # center is 0.03282 m above it, while the measured radius places the
    # ground below the wheel center. Derive the crop box in this live frame.
    wheel_center_z = calibration["base_link"]["husky_wheel_center"]["z"]
    ground_z = wheel_center_z - vehicle["wheel_radius"]
    half_wheel_base = vehicle["wheel_base"] / 2.0
    crop_parameters = {
        "input_frame": "base_link",
        "output_frame": "base_link",
        "negative": True,
        "processing_time_threshold_sec": 0.01,
        "min_x": -(half_wheel_base + vehicle["rear_overhang"]),
        "max_x": half_wheel_base + vehicle["front_overhang"],
        "min_y": -(vehicle["wheel_tread"] / 2.0 + vehicle["right_overhang"]),
        "max_y": vehicle["wheel_tread"] / 2.0 + vehicle["left_overhang"],
        "min_z": ground_z,
        "max_z": ground_z + vehicle["vehicle_height"],
    }

    components = [
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_self",
            remappings=[
                ("input", "/sensing/lidar/top/pointcloud_raw_ex"),
                ("output", "/sensing/lidar/top/self_cropped/pointcloud_ex"),
            ],
            parameters=[crop_parameters],
        ),
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::DistortionCorrectorComponent",
            name="distortion_corrector_node",
            remappings=[
                ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
                ("~/input/imu", "/sensing/imu/imu_data"),
                ("~/input/pointcloud", "/sensing/lidar/top/self_cropped/pointcloud_ex"),
                ("~/output/pointcloud", "/sensing/lidar/top/rectified/pointcloud_ex"),
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
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::RingOutlierFilterComponent",
            name="ring_outlier_filter",
            remappings=[
                ("input", "/sensing/lidar/top/rectified/pointcloud_ex"),
                ("output", "/sensing/lidar/concatenated/pointcloud"),
            ],
            parameters=[{
                "distance_ratio": 1.03,
                "object_length_threshold": 0.05,
                "max_rings_num": 16,
                "max_points_num_per_ring": 4000,
                "publish_outlier_pointcloud": False,
                "min_azimuth_deg": 0.0,
                "max_azimuth_deg": 360.0,
                "max_distance": 12.0,
                "vertical_bins": 16,
                "horizontal_bins": 36,
                "noise_threshold": 2,
                "processing_time_threshold_sec": 0.01,
                "output_frame": "velodyne",
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

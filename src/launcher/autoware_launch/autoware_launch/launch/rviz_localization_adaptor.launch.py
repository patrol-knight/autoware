"""Connect RViz's 2D Pose Estimate tool to localization without the full AD API."""

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            ComposableNodeContainer(
                package="rclcpp_components",
                executable="component_container_mt",
                namespace="/adapi",
                name="localization_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="autoware_default_adapi",
                        plugin="autoware::default_adapi::LocalizationNode",
                        namespace="/adapi/node",
                        name="localization",
                        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                    )
                ],
                output="screen",
            ),
            Node(
                package="autoware_adapi_adaptors",
                executable="initial_pose_adaptor_node",
                namespace="/default_adapi/helpers",
                name="autoware_initial_pose_adaptor",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    PathJoinSubstitution(
                        [FindPackageShare("autoware_adapi_adaptors"), "config", "initial_pose.param.yaml"]
                    ),
                    {
                        "map_height_fitter.map_loader_name": "/map/pointcloud_map_loader",
                        "map_height_fitter.target": "pointcloud_map",
                    },
                ],
                remappings=[
                    ("~/initialpose", "/initialpose"),
                    ("~/pointcloud_map", "/map/pointcloud_map"),
                    ("~/partial_map_load", "/map/get_partial_pointcloud_map"),
                    ("~/vector_map", "/map/vector_map"),
                ],
                output="screen",
            ),
            Node(
                package="autoware_launch",
                executable="localization_state_logger.py",
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                output="screen",
            ),
        ]
    )

#!/usr/bin/env python3
"""Publish an obstacle stop decision from each VLP-16 scan."""

from time import perf_counter_ns
from math import isfinite

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float64, Int32

from lidar_obstacle_core import count_front_obstacles, segment_ground


class LidarObstacleStop(Node):
    def __init__(self):
        super().__init__("lidar_obstacle_stop")
        self.declare_parameter("input_topic", "/sensing/lidar/concatenated/pointcloud")
        self.declare_parameter("sensor_frame", "velodyne")
        self.declare_parameter("detection_fps", 10.0)
        self.declare_parameter("num_scans", 16)
        self.declare_parameter("horizon_scan", 1800)
        self.declare_parameter("ground_scan_index", 7)
        self.declare_parameter("sensor_mount_angle_deg", 0.0)
        self.declare_parameter("ground_angle_tolerance_deg", 10.0)
        self.declare_parameter("max_range_m", 2.0)
        self.declare_parameter("half_angle_deg", 60.0)
        self.declare_parameter("point_threshold", 20)
        self.sensor_frame = self.get_parameter("sensor_frame").value
        self.detection_fps = float(self.get_parameter("detection_fps").value)
        self.num_scans = int(self.get_parameter("num_scans").value)
        self.horizon_scan = int(self.get_parameter("horizon_scan").value)
        self.ground_scan_index = int(self.get_parameter("ground_scan_index").value)
        self.sensor_mount_angle = float(self.get_parameter("sensor_mount_angle_deg").value)
        self.ground_angle_tolerance = float(
            self.get_parameter("ground_angle_tolerance_deg").value
        )
        self.max_range = float(self.get_parameter("max_range_m").value)
        self.half_angle = float(self.get_parameter("half_angle_deg").value)
        self.threshold = int(self.get_parameter("point_threshold").value)
        if (self.num_scans <= 1 or self.horizon_scan <= 0 or
                not 0 <= self.ground_scan_index < self.num_scans - 1 or
                self.ground_angle_tolerance < 0 or self.max_range <= 0 or
                not 0 < self.half_angle <= 90 or self.threshold <= 0 or
                not isfinite(self.detection_fps) or self.detection_fps <= 0):
            raise ValueError("Invalid LiDAR obstacle stop parameters")
        self.stop_pub = self.create_publisher(Bool, "/sensing/lidar/obstacle_stop", 10)
        self.count_pub = self.create_publisher(Int32, "/sensing/lidar/obstacle_count", 10)
        self.runtime_pub = self.create_publisher(
            Float64, "/sensing/lidar/obstacle_exe_time_ms", 10
        )
        self.ground_pub = self.create_publisher(
            PointCloud2, "/sensing/lidar/ground/pointcloud", qos_profile_sensor_data
        )
        self.nonground_pub = self.create_publisher(
            PointCloud2, "/sensing/lidar/nonground/pointcloud", qos_profile_sensor_data
        )
        self.pending_cloud = None
        self.create_subscription(
            PointCloud2, self.get_parameter("input_topic").value,
            self.on_cloud, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.create_timer(
            1.0 / self.detection_fps, self.on_detection_timer,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def on_cloud(self, cloud):
        # Retain only the newest corrected scan when input is faster than detection_fps.
        self.pending_cloud = cloud

    def on_detection_timer(self):
        cloud = self.pending_cloud
        self.pending_cloud = None
        if cloud is None:
            return
        self.process_cloud(cloud)

    def process_cloud(self, cloud):
        start_ns = perf_counter_ns()
        if cloud.header.frame_id != self.sensor_frame:
            self.get_logger().error(
                f"Ignoring cloud frame {cloud.header.frame_id!r}; expected {self.sensor_frame!r}",
                throttle_duration_sec=5.0,
            )
            return
        fields = {field.name for field in cloud.fields}
        channel_field = "channel" if "channel" in fields else "ring"
        if not {"x", "y", "z", channel_field}.issubset(fields):
            self.get_logger().error(
                "LiDAR cloud lacks x/y/z and channel (or ring) fields",
                throttle_duration_sec=5.0,
            )
            return
        try:
            points = point_cloud2.read_points(cloud, field_names=("x", "y", "z", channel_field),
                                               skip_nans=False)
            ground, nonground = segment_ground(
                points, num_scans=self.num_scans,
                horizon_scan=self.horizon_scan,
                ground_scan_index=self.ground_scan_index,
                sensor_mount_angle_deg=self.sensor_mount_angle,
                ground_angle_tolerance_deg=self.ground_angle_tolerance,
                half_angle_deg=self.half_angle,
            )
            count = count_front_obstacles(
                nonground, max_range=self.max_range, half_angle_deg=self.half_angle,
                threshold=self.threshold,
            )
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().error(f"Invalid LiDAR cloud: {exc}", throttle_duration_sec=5.0)
            return
        if self.ground_pub.get_subscription_count():
            self.ground_pub.publish(point_cloud2.create_cloud_xyz32(cloud.header, ground))
        if self.nonground_pub.get_subscription_count():
            self.nonground_pub.publish(point_cloud2.create_cloud_xyz32(cloud.header, nonground))
        self.count_pub.publish(Int32(data=count))
        self.stop_pub.publish(Bool(data=count >= self.threshold))
        self.runtime_pub.publish(Float64(data=(perf_counter_ns() - start_ns) / 1_000_000.0))


def main():
    rclpy.init()
    node = LidarObstacleStop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

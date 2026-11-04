#!/usr/bin/env python3
"""Publish VLP-16 data on Autoware's raw LiDAR input."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class PointCloudBridge(Node):
    def __init__(self):
        super().__init__("husky_pointcloud_bridge")
        self.raw_publisher = self.create_publisher(
            PointCloud2, "/sensing/lidar/top/pointcloud_raw_ex", qos_profile_sensor_data
        )
        self.declare_parameter("publish_concatenated", False)
        self.declare_parameter("lidar_input_rate", 0.0)
        self.lidar_input_rate = self.get_parameter("lidar_input_rate").value
        if not math.isfinite(self.lidar_input_rate) or self.lidar_input_rate < 0.0:
            raise ValueError("lidar_input_rate must be finite and nonnegative")
        self.concatenated_publisher = None
        if self.get_parameter("publish_concatenated").value:
            self.concatenated_publisher = self.create_publisher(
                PointCloud2, "/sensing/lidar/concatenated/pointcloud", qos_profile_sensor_data
            )
        self.last_accepted_stamp_ns = None
        self.last_accepted_at = None
        self.subscription = self.create_subscription(
            PointCloud2, "/velodyne_points", self.on_cloud, qos_profile_sensor_data
        )

    def on_cloud(self, msg):
        self.raw_publisher.publish(msg)
        if self.concatenated_publisher is None:
            return
        if self.concatenated_publisher.get_subscription_count() == 0:
            return

        if self.lidar_input_rate > 0.0:
            now = time.monotonic()
            interval_ns = 1e9 / self.lidar_input_rate
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if stamp_ns and self.last_accepted_stamp_ns is not None:
                elapsed_ns = stamp_ns - self.last_accepted_stamp_ns
                if 0 <= elapsed_ns < interval_ns:
                    return
            elif self.last_accepted_at is not None:
                if now - self.last_accepted_at < 1.0 / self.lidar_input_rate:
                    return
            self.last_accepted_stamp_ns = stamp_ns if stamp_ns else None
            self.last_accepted_at = now

        self.concatenated_publisher.publish(msg)


def main():
    rclpy.init()
    node = PointCloudBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

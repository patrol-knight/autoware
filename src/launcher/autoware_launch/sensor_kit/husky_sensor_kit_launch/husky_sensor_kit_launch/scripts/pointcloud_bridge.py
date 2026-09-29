#!/usr/bin/env python3
"""Publish VLP-16 data on Autoware's raw LiDAR input."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class PointCloudBridge(Node):
    def __init__(self):
        super().__init__("husky_pointcloud_bridge")
        self.raw_publisher = self.create_publisher(
            PointCloud2, "/sensing/lidar/top/pointcloud_raw_ex", 10
        )
        self.subscription = self.create_subscription(
            PointCloud2, "/velodyne_points", self.on_cloud, qos_profile_sensor_data
        )

    def on_cloud(self, msg):
        self.raw_publisher.publish(msg)


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

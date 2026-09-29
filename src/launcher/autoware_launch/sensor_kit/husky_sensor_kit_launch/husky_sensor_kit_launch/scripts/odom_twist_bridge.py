#!/usr/bin/env python3
"""Expose Husky's filtered wheel odometry twist to Autoware localization."""

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class OdomTwistBridge(Node):
    def __init__(self):
        super().__init__("husky_odom_twist_bridge")
        self.publisher = self.create_publisher(
            TwistWithCovarianceStamped,
            "/sensing/vehicle_velocity_converter/twist_with_covariance",
            10,
        )
        self.subscription = self.create_subscription(
            Odometry, "/husky/platform/odom/filtered", self.on_odom, qos_profile_sensor_data
        )

    def on_odom(self, msg):
        output = TwistWithCovarianceStamped()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = msg.child_frame_id or "base_link"
        output.twist = msg.twist
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = OdomTwistBridge()
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

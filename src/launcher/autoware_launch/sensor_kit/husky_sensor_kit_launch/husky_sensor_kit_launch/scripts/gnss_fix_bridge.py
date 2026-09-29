#!/usr/bin/env python3
"""Adapt Fixposition's sensor-data QoS to the GNSS poser's input QoS."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix


class GnssFixBridge(Node):
    def __init__(self):
        super().__init__("fixposition_gnss_fix_bridge")
        self.publisher = self.create_publisher(NavSatFix, "/sensing/gnss/fix", 10)
        self.subscription = self.create_subscription(
            NavSatFix, "/fixposition/gnss1", self.on_fix, qos_profile_sensor_data
        )

    def on_fix(self, msg):
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = GnssFixBridge()
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

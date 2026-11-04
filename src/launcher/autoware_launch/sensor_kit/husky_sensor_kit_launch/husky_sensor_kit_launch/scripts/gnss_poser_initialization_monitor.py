#!/usr/bin/env python3
"""Exit successfully when localization is initialized, signaling the launch file."""

import rclpy
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class InitializationMonitor(Node):
    def __init__(self):
        super().__init__("gnss_poser_initialization_monitor")
        self.initialized = False
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.subscription = self.create_subscription(
            LocalizationInitializationState,
            "/localization/initialization_state",
            self.on_state,
            qos,
        )

    def on_state(self, msg):
        if msg.state == LocalizationInitializationState.INITIALIZED:
            self.get_logger().info("Localization initialized; stopping GNSS poser")
            self.initialized = True


def main():
    rclpy.init()
    node = InitializationMonitor()
    try:
        while rclpy.ok() and not node.initialized:
            rclpy.spin_once(node)
        return 0 if node.initialized else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

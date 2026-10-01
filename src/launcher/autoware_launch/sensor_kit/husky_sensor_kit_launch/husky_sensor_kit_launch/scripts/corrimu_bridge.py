#!/usr/bin/env python3
"""Expose Fixposition CORRIMU's nested sensor_msgs/Imu to Autoware."""

import math

import rclpy
from fixposition_driver_msgs.msg import FpaImu
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


# Conservative starting estimate: 0.02 rad/s standard deviation on each axis.
ANGULAR_VELOCITY_VARIANCE_FLOOR = 0.02**2


class CorrimuBridge(Node):
    def __init__(self):
        super().__init__("fixposition_corrimu_bridge")
        self.publisher = self.create_publisher(Imu, "/sensing/imu/imu_data", 10)
        self.subscription = self.create_subscription(
            FpaImu, "/fixposition/fpa/corrimu", self.on_corrimu, qos_profile_sensor_data
        )

    def on_corrimu(self, msg):
        imu = msg.data
        if imu.header.frame_id != "FP_VRTK":
            self.get_logger().warn(
                f"Unexpected CORRIMU frame {imu.header.frame_id}; dropping sample",
                throttle_duration_sec=5.0,
            )
            return
        # CORRIMU has angular velocity and acceleration, but no orientation.
        imu.orientation_covariance[0] = -1.0
        # The driver currently reports zero angular-velocity covariance. Keep
        # valid, larger device estimates while ensuring EKF measurements have
        # nonzero uncertainty.
        for index in (0, 4, 8):
            variance = imu.angular_velocity_covariance[index]
            if not math.isfinite(variance) or variance < ANGULAR_VELOCITY_VARIANCE_FLOOR:
                imu.angular_velocity_covariance[index] = ANGULAR_VELOCITY_VARIANCE_FLOOR
        self.publisher.publish(imu)


def main():
    rclpy.init()
    node = CorrimuBridge()
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

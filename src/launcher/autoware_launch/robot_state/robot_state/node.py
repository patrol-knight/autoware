#!/usr/bin/env python3
"""Summarize localization, routing and control authority as std_msgs/String."""

import time
from math import isfinite

import rclpy
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState, OperationModeState
from autoware_planning_msgs.msg import RouteState
from autoware_vehicle_msgs.msg import VelocityReport
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Header, String

REACH_DELIVERY_S = 0.2


class RobotState(Node):
    def __init__(self):
        super().__init__("robot_state")
        self.declare_parameter("sensor_timeout_s", 5.0)
        self.declare_parameter("velocity_timeout_s", 1.0)
        self.declare_parameter("stop_linear_tolerance_mps", 0.01)
        self.declare_parameter("stop_angular_tolerance_radps", 0.01)
        self.declare_parameter("lidar_topic", "/sensing/lidar/obstacle_stop")
        self.declare_parameter("imu_topic", "/sensing/imu/imu_data")
        self.declare_parameter("wheel_odometry_topic", "/husky/platform/odom")
        self.sensor_timeout_s = float(self.get_parameter("sensor_timeout_s").value)
        self.velocity_timeout_s = float(self.get_parameter("velocity_timeout_s").value)
        self.stop_linear_tolerance_mps = float(
            self.get_parameter("stop_linear_tolerance_mps").value
        )
        self.stop_angular_tolerance_radps = float(
            self.get_parameter("stop_angular_tolerance_radps").value
        )
        if (self.sensor_timeout_s <= 0 or self.velocity_timeout_s <= 0
                or self.stop_linear_tolerance_mps < 0
                or self.stop_angular_tolerance_radps < 0):
            raise ValueError("Status timeouts must be positive and stop tolerances nonnegative")
        self.localization = None
        self.route = None
        self.operation = None
        self.failed = False
        self.retry_failure_active = False
        self.cancelled = False
        self.reach_published_for_arrival = False
        self.reach_published_at = None
        self.arrived_at = None
        self.velocity_report = None
        self.velocity_seen_at = None
        self.sensor_last_seen = {"lidar": None, "imu": None, "wheel_odometry": None}
        self.readiness_established = False
        self.last_status = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, "/autoware/robot_status", qos)
        self.create_subscription(
            LocalizationInitializationState,
            "/localization/initialization_state", self.on_localization, qos,
        )
        self.create_subscription(RouteState, "/planning/route_state", self.on_route, qos)
        self.create_subscription(Header, "/husky/cancel", self.on_cancel, 10)
        self.create_subscription(
            OperationModeState, "/system/operation_mode/state", self.on_operation, qos,
        )
        self.create_subscription(
            String, "/planning/route_request_status", self.on_route_request, 10,
        )
        self.create_subscription(
            VelocityReport, "/vehicle/status/velocity_status", self.on_velocity, 10,
        )
        for name, message_type, parameter in (
            ("lidar", Bool, "lidar_topic"),
            ("imu", Imu, "imu_topic"),
            ("wheel_odometry", Odometry, "wheel_odometry_topic"),
        ):
            topic = str(self.get_parameter(parameter).value)
            if not topic.startswith("/"):
                raise ValueError(f"{parameter} must be an absolute topic name")
            self.create_subscription(
                message_type, topic,
                lambda msg, name=name: self.on_sensor(name, msg),
                qos_profile_sensor_data,
            )
        self.status_timer = self.create_timer(0.2, lambda: self.publish_status(force=True))
        self.publish_status()

    def on_localization(self, msg):
        self.localization = msg.state
        self.publish_status()

    def on_route(self, msg):
        if msg.state == RouteState.UNSET:
            self.cancelled = False
        elif self.cancelled:
            return
        if msg.state == RouteState.ARRIVED and self.route != RouteState.ARRIVED:
            self.reach_published_for_arrival = False
            self.reach_published_at = None
            self.arrived_at = time.monotonic()
        elif msg.state != RouteState.ARRIVED:
            self.reach_published_for_arrival = False
            self.reach_published_at = None
            self.arrived_at = None
        self.route = msg.state
        if (msg.state in (RouteState.SET, RouteState.ROUTING, RouteState.REROUTING)
                and not self.retry_failure_active):
            self.failed = False
        elif msg.state == RouteState.ABORTED:
            self.failed = True
        self.publish_status()

    def on_operation(self, msg):
        self.operation = msg
        self.publish_status()

    def on_route_request(self, msg):
        if msg.data == "cancelled":
            self.on_cancel(None)
            return
        if self.cancelled:
            return
        if msg.data == "rejected":
            self.failed = True
            self.retry_failure_active = True
        elif msg.data in ("pending", "accepted"):
            self.failed = False
            self.retry_failure_active = False
        self.publish_status()

    def on_cancel(self, _msg):
        self.cancelled = self.route != RouteState.UNSET or self.failed
        self.failed = False
        self.retry_failure_active = False
        self.reach_published_for_arrival = False
        self.reach_published_at = None
        self.arrived_at = None
        self.publish_status()

    def on_velocity(self, msg):
        values = (msg.longitudinal_velocity, msg.lateral_velocity, msg.heading_rate)
        if not all(isfinite(value) for value in values):
            return
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return
        age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
        if not -0.2 <= age <= self.velocity_timeout_s:
            return
        self.velocity_report = msg
        self.velocity_seen_at = time.monotonic()
        self.publish_status()

    def vehicle_stopped(self, now):
        msg = self.velocity_report
        return (
            msg is not None
            and self.arrived_at is not None
            and self.velocity_seen_at is not None
            and self.velocity_seen_at >= self.arrived_at
            and now - self.velocity_seen_at <= self.velocity_timeout_s
            and abs(msg.longitudinal_velocity) <= self.stop_linear_tolerance_mps
            and abs(msg.lateral_velocity) <= self.stop_linear_tolerance_mps
            and abs(msg.heading_rate) <= self.stop_angular_tolerance_radps
        )

    def on_sensor(self, name, _msg):
        self.sensor_last_seen[name] = time.monotonic()
        self.publish_status()

    def status(self):
        if self.failed:
            return "failure"
        now = time.monotonic()
        if not self.readiness_established:
            if self.localization != LocalizationInitializationState.INITIALIZED or any(
                seen is None or now - seen > self.sensor_timeout_s
                for seen in self.sensor_last_seen.values()
            ):
                return "notready"
            self.readiness_established = True
        if self.cancelled:
            return "ready"
        if self.route == RouteState.ARRIVED:
            if not self.reach_published_for_arrival and self.vehicle_stopped(now):
                return "reach"
            if (self.last_status == "reach" and self.reach_published_at is not None
                    and now - self.reach_published_at < REACH_DELIVERY_S):
                return "reach"
            return "ready"
        if (self.operation is not None
                and self.operation.mode == OperationModeState.AUTONOMOUS
                and self.operation.is_autoware_control_enabled
                and self.route in (RouteState.SET, RouteState.REROUTING)):
            return "navigate"
        return "ready"

    def publish_status(self, force=False):
        status = self.status()
        changed = status != self.last_status
        if not changed and (not force or status == "reach"):
            return
        self.publisher.publish(String(data=status))
        if status == "reach":
            self.reach_published_for_arrival = True
            self.reach_published_at = time.monotonic()
        self.last_status = status
        if changed:
            self.get_logger().info(f"Robot status: {status}")


def main():
    rclpy.init()
    node = RobotState()
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

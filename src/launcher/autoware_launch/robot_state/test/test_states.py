import time
from unittest.mock import Mock

import rclpy
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState, OperationModeState
from autoware_planning_msgs.msg import RouteState
from autoware_vehicle_msgs.msg import VelocityReport
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String

from robot_state.node import REACH_DELIVERY_S, RobotState


def feed_sensors(node):
    node.on_sensor("lidar", PointCloud2())
    node.on_sensor("imu", Imu())
    node.on_sensor("wheel_odometry", Odometry())


def feed_velocity(node, longitudinal=0.0, lateral=0.0, heading_rate=0.0):
    msg = VelocityReport()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.longitudinal_velocity = longitudinal
    msg.lateral_velocity = lateral
    msg.heading_rate = heading_rate
    node.on_velocity(msg)


def test_steady_status_republishes_at_timer_ticks_except_reach():
    rclpy.init()
    node = RobotState()
    try:
        assert node.status_timer.timer_period_ns == 200_000_000
        node.publisher = Mock()
        node.publish_status(force=True)
        node.publish_status(force=True)
        assert [call.args[0].data for call in node.publisher.publish.call_args_list] == [
            "notready", "notready"
        ]

        node.on_localization(
            LocalizationInitializationState(state=LocalizationInitializationState.INITIALIZED)
        )
        feed_sensors(node)
        node.publisher.reset_mock()
        node.publish_status(force=True)
        node.publish_status(force=True)
        assert [call.args[0].data for call in node.publisher.publish.call_args_list] == [
            "ready", "ready"
        ]

        node.publisher.reset_mock()
        node.on_route(RouteState(state=RouteState.ARRIVED))
        feed_velocity(node)
        node.publish_status(force=True)
        node.publish_status(force=True)
        assert [call.args[0].data for call in node.publisher.publish.call_args_list] == [
            "reach"
        ]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_navigation_lifecycle():
    rclpy.init()
    node = RobotState()
    try:
        assert node.status() == "notready"
        node.on_localization(
            LocalizationInitializationState(state=LocalizationInitializationState.INITIALIZED)
        )
        assert node.status() == "notready"
        feed_sensors(node)
        assert node.status() == "ready"

        node.on_route_request(String(data="pending"))
        assert node.status() == "ready"

        node.on_route(RouteState(state=RouteState.ROUTING))
        assert node.status() == "ready"
        node.on_route(RouteState(state=RouteState.SET))
        node.on_operation(OperationModeState(mode=OperationModeState.STOP))
        assert node.status() == "ready"
        node.on_operation(
            OperationModeState(
                mode=OperationModeState.AUTONOMOUS,
                is_autoware_control_enabled=True,
            )
        )
        assert node.status() == "navigate"

        node.publisher = Mock()
        node.on_route(RouteState(state=RouteState.ARRIVED))
        node.on_operation(OperationModeState(mode=OperationModeState.LOCAL))
        assert node.status() == "ready"
        feed_velocity(node, longitudinal=0.1)
        assert node.status() == "ready"
        feed_velocity(node)
        assert node.status() == "reach"
        assert node.reach_published_for_arrival
        assert [call.args[0].data for call in node.publisher.publish.call_args_list] == [
            "ready", "reach"
        ]
        node.on_route(RouteState(state=RouteState.ARRIVED))
        assert node.publisher.publish.call_count == 2
        node.reach_published_at = time.monotonic() - REACH_DELIVERY_S - 0.01
        node.publish_status()
        assert [call.args[0].data for call in node.publisher.publish.call_args_list] == [
            "ready", "reach", "ready"
        ]
        node.on_manual_command(Twist())
        assert node.status() == "manual"
        node.manual_command_at = time.monotonic() - node.manual_command_timeout_s - 1
        assert node.status() == "ready"
        node.on_route_request(String(data="pending"))
        assert node.status() == "ready"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_reach_is_published_once_per_arrival_even_after_manual_interruption():
    rclpy.init()
    node = RobotState()
    try:
        node.on_localization(
            LocalizationInitializationState(state=LocalizationInitializationState.INITIALIZED)
        )
        feed_sensors(node)
        node.on_operation(OperationModeState(mode=OperationModeState.LOCAL))
        node.publisher = Mock()
        node.on_route(RouteState(state=RouteState.ARRIVED))
        feed_velocity(node)
        assert [call.args[0].data for call in node.publisher.publish.call_args_list] == [
            "reach"
        ]

        node.on_manual_command(Twist())
        assert node.last_status == "manual"
        node.manual_command_at = time.monotonic() - node.manual_command_timeout_s - 1
        node.publish_status()
        assert node.last_status == "ready"
        node.on_route(RouteState(state=RouteState.ARRIVED))
        assert node.last_status == "ready"
        assert sum(
            call.args[0].data == "reach"
            for call in node.publisher.publish.call_args_list
        ) == 1

        node.on_route(RouteState(state=RouteState.UNSET))
        node.on_route(RouteState(state=RouteState.ARRIVED))
        feed_velocity(node)
        assert node.last_status == "reach"
        assert sum(
            call.args[0].data == "reach"
            for call in node.publisher.publish.call_args_list
        ) == 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_reach_waits_for_new_zero_velocity_in_all_axes():
    rclpy.init()
    node = RobotState()
    try:
        node.on_localization(
            LocalizationInitializationState(state=LocalizationInitializationState.INITIALIZED)
        )
        feed_sensors(node)
        node.publisher = Mock()
        feed_velocity(node)
        node.on_route(RouteState(state=RouteState.ARRIVED))
        assert node.status() == "ready"

        for velocities in (
            (0.02, 0.0, 0.0),
            (0.0, 0.02, 0.0),
            (0.0, 0.0, 0.02),
        ):
            feed_velocity(node, *velocities)
            assert node.status() == "ready"
        assert all(
            call.args[0].data != "reach"
            for call in node.publisher.publish.call_args_list
        )

        feed_velocity(node)
        assert node.status() == "reach"
        assert sum(
            call.args[0].data == "reach"
            for call in node.publisher.publish.call_args_list
        ) == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_failure_and_manual_priority():
    rclpy.init()
    node = RobotState()
    try:
        node.on_localization(
            LocalizationInitializationState(state=LocalizationInitializationState.INITIALIZED)
        )
        feed_sensors(node)
        node.on_operation(OperationModeState(mode=OperationModeState.LOCAL))
        assert node.status() == "ready"
        node.on_manual_command(Twist())
        assert node.status() == "manual"
        node.on_route_request(String(data="rejected"))
        assert node.status() == "failure"
        node.on_route(RouteState(state=RouteState.ABORTED))
        assert node.status() == "failure"
        node.on_route(RouteState(state=RouteState.SET))
        assert node.status() == "failure"
        node.manual_command_at = time.monotonic() - node.manual_command_timeout_s - 1
        assert node.status() == "failure"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_notready_manual_returns_to_notready_without_route_cancel():
    rclpy.init()
    node = RobotState()
    try:
        node.manual_route_cancel_pub = Mock()
        node.on_operation(OperationModeState(mode=OperationModeState.LOCAL))
        node.on_manual_command(Twist())
        assert node.status() == "manual"
        node.manual_route_cancel_pub.publish.assert_not_called()
        node.manual_command_at = time.monotonic() - node.manual_command_timeout_s - 1
        node.publish_status()
        assert node.status() == "notready"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_manual_from_active_route_requests_clear_once_and_returns_ready():
    rclpy.init()
    node = RobotState()
    try:
        node.manual_route_cancel_pub = Mock()
        node.on_localization(
            LocalizationInitializationState(state=LocalizationInitializationState.INITIALIZED)
        )
        feed_sensors(node)
        node.on_route(RouteState(state=RouteState.SET))
        node.on_operation(OperationModeState(
            mode=OperationModeState.AUTONOMOUS, is_autoware_control_enabled=True
        ))
        assert node.status() == "navigate"
        node.on_operation(OperationModeState(mode=OperationModeState.LOCAL))
        node.on_manual_command(Twist())
        assert node.status() == "manual"
        assert node.manual_route_cancel_pub.publish.call_count == 1
        assert node.manual_route_cancel_pub.publish.call_args.args[0].data == "ready"
        node.on_manual_command(Twist())
        assert node.manual_route_cancel_pub.publish.call_count == 1
        node.on_route(RouteState(state=RouteState.UNSET))
        node.manual_command_at = time.monotonic() - node.manual_command_timeout_s - 1
        node.publish_status()
        assert node.status() == "ready"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_failure_survives_retry_route_until_recovery_event():
    rclpy.init()
    node = RobotState()
    try:
        node.on_localization(
            LocalizationInitializationState(state=LocalizationInitializationState.INITIALIZED)
        )
        feed_sensors(node)
        node.on_route_request(String(data="rejected"))
        node.on_route(RouteState(state=RouteState.ROUTING))
        node.on_route(RouteState(state=RouteState.SET))
        node.on_route(RouteState(state=RouteState.UNSET))
        assert node.status() == "failure"
        node.on_route_request(String(data="accepted"))
        assert node.status() == "ready"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_each_missing_or_stale_sensor_reports_notready():
    rclpy.init()
    node = RobotState()
    try:
        node.on_localization(
            LocalizationInitializationState(state=LocalizationInitializationState.INITIALIZED)
        )
        assert node.status() == "notready"
        for name, message in (
            ("lidar", PointCloud2()),
            ("imu", Imu()),
            ("wheel_odometry", Odometry()),
        ):
            node.on_sensor(name, message)
            if name != "wheel_odometry":
                assert node.status() == "notready"
        assert node.status() == "ready"

        for name in node.sensor_last_seen:
            node.sensor_last_seen[name] = time.monotonic() - node.sensor_timeout_s - 1
            node.publish_status()
            assert node.status() == "notready"
            feed_sensors(node)
            assert node.status() == "ready"
    finally:
        node.destroy_node()
        rclpy.shutdown()

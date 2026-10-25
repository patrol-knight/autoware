#!/usr/bin/env python3
"""Explicitly armed Husky Twist bridge with manual and E-stop interlocks."""

import signal
import time
from math import isfinite

import rclpy
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode
from autoware_planning_msgs.msg import RouteState
from autoware_vehicle_msgs.msg import ControlModeReport, VelocityReport
from autoware_vehicle_msgs.srv import ControlModeCommand
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from husky_twist_gate_core import TwistGate


class HuskyTwistVehicleInterface(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__(
            "husky_twist_vehicle_interface", parameter_overrides=parameter_overrides
        )
        self.declare_parameter("input_topic", "/control/husky/cmd_vel")
        self.declare_parameter("output_topic", "/husky/cmd_vel")
        self.declare_parameter("estop_topic", "/husky/platform/emergency_stop")
        self.declare_parameter("odometry_topic", "/husky/platform/odom/filtered")
        self.declare_parameter("velocity_frame_id", "base_link")
        self.declare_parameter("max_odometry_age_s", 0.7)
        self.declare_parameter("tracking_state_topic", "/control/husky/tracking_state")
        self.declare_parameter("tracking_timeout_s", 0.5)
        self.declare_parameter("manual_topics", [
            "/husky/joy_teleop/cmd_vel",
            "/husky/rc_teleop/cmd_vel",
            "/husky/twist_marker_server/cmd_vel",
        ])
        self.declare_parameter("remote_manual_topics", [
            "/husky/joy_teleop/cmd_vel",
            "/husky/rc_teleop/cmd_vel",
        ])
        self.declare_parameter("allow_platform_output", False)
        self.declare_parameter("rviz_operation_mode_api", False)
        self.declare_parameter("command_timeout_s", 0.25)
        self.declare_parameter("estop_timeout_s", 2.0)
        self.declare_parameter("manual_hold_s", 1.0)
        self.declare_parameter("max_linear_speed", 1.0)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("stop_publish_duration_s", 0.6)
        self.declare_parameter("obstacle_stop_topic", "/sensing/lidar/obstacle_stop")
        self.declare_parameter("obstacle_timeout_s", 1.5)
        self.declare_parameter("require_obstacle_status", False)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        estop_topic = str(self.get_parameter("estop_topic").value)
        odometry_topic = str(self.get_parameter("odometry_topic").value)
        tracking_state_topic = str(self.get_parameter("tracking_state_topic").value)
        self.velocity_frame_id = str(self.get_parameter("velocity_frame_id").value)
        self.max_odometry_age_s = float(self.get_parameter("max_odometry_age_s").value)
        manual_topics = list(self.get_parameter("manual_topics").value)
        remote_manual_topics = set(self.get_parameter("remote_manual_topics").value)
        if not all(topic.startswith("/") for topic in [input_topic, output_topic, estop_topic, odometry_topic, tracking_state_topic, *manual_topics]):
            raise ValueError("Husky interface topics must use absolute names")
        if len(set([input_topic, output_topic, estop_topic, odometry_topic, tracking_state_topic, *manual_topics])) != 5 + len(manual_topics):
            raise ValueError("Husky interface topics must differ")
        if not manual_topics:
            raise ValueError("At least one manual takeover topic is required")
        if not remote_manual_topics.issubset(manual_topics):
            raise ValueError("Remote manual topics must be a subset of manual topics")
        self.local_manual_topics = set(manual_topics) - remote_manual_topics
        self.remote_manual_topics = remote_manual_topics
        if not self.velocity_frame_id or self.max_odometry_age_s <= 0.0:
            raise ValueError("Velocity frame and maximum odometry age must be valid")

        allow_output = bool(self.get_parameter("allow_platform_output").value)
        if self._simulated_time_active():
            allow_output = False
            self.get_logger().warn("Simulated time is active; platform output is inhibited")
        self.gate = TwistGate(
            allow_output=allow_output,
            command_timeout=float(self.get_parameter("command_timeout_s").value),
            estop_timeout=float(self.get_parameter("estop_timeout_s").value),
            odometry_timeout=self.max_odometry_age_s,
            tracking_timeout=float(self.get_parameter("tracking_timeout_s").value),
            manual_hold=float(self.get_parameter("manual_hold_s").value),
            max_linear=float(self.get_parameter("max_linear_speed").value),
            max_angular=float(self.get_parameter("max_angular_speed").value),
        )
        self.stop_duration = float(self.get_parameter("stop_publish_duration_s").value)
        rate = float(self.get_parameter("publish_rate_hz").value)
        if self.stop_duration <= 0.0 or rate <= 0.0:
            raise ValueError("Stop duration and publish rate must be positive")
        self.period = 1.0 / rate
        self.stop_until = None
        self.last_state = None
        self.output_topic = output_topic
        self.require_obstacle_status = bool(self.get_parameter("require_obstacle_status").value)
        self.obstacle_timeout = float(self.get_parameter("obstacle_timeout_s").value)
        if self.obstacle_timeout <= 0.0:
            raise ValueError("Obstacle timeout must be positive")
        self.obstacle_stop = True
        self.obstacle_at = None

        self.output_pub = self.create_publisher(Twist, output_topic, 10)
        self.state_pub = self.create_publisher(String, "/control/husky/bridge_state", 10)
        self.velocity_pub = self.create_publisher(
            VelocityReport, "/vehicle/status/velocity_status", 10
        )
        self.mode_pub = self.create_publisher(
            ControlModeReport, "/vehicle/status/control_mode", 10
        )
        self.operation_mode_pub = self.create_publisher(
            OperationModeState,
            "/system/operation_mode/state",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.rviz_operation_mode_api = bool(
            self.get_parameter("rviz_operation_mode_api").value
        )
        self.api_operation_mode_pub = None
        if self.rviz_operation_mode_api:
            self.api_operation_mode_pub = self.create_publisher(
                OperationModeState,
                "/api/operation_mode/state",
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
            for service_name, callback in (
                ("change_to_autonomous", self.on_api_autonomous),
                ("change_to_stop", self.on_api_stop),
                ("change_to_local", self.on_api_local),
                ("change_to_remote", self.on_api_remote),
                ("enable_autoware_control", self.on_api_autonomous),
                ("disable_autoware_control", self.on_api_disable),
            ):
                self.create_service(
                    ChangeOperationMode,
                    f"/api/operation_mode/{service_name}",
                    callback,
                )
        self.create_subscription(Twist, input_topic, self.on_command, 10)
        self.create_subscription(Bool, estop_topic, self.on_estop, qos_profile_sensor_data)
        self.create_subscription(
            Bool, str(self.get_parameter("obstacle_stop_topic").value),
            self.on_obstacle_stop, 10,
        )
        self.create_subscription(
            Odometry, odometry_topic, self.on_odometry, qos_profile_sensor_data
        )
        self.create_subscription(String, tracking_state_topic, self.on_tracking, 10)
        self.route_state_sub = self.create_subscription(
            RouteState,
            "/planning/route_state",
            self.on_route_state,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.manual_subs = [
            self.create_subscription(
                Twist, topic,
                lambda msg, topic=topic: self.on_manual(msg, topic), 10
            )
            for topic in manual_topics
        ]
        self.create_service(SetBool, "/control/husky/set_enabled", self.on_enable_request)
        self.create_service(
            ControlModeCommand,
            "/control/control_mode_request",
            self.on_control_mode_request,
        )
        self.create_timer(
            self.period, self.on_timer, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        self.get_logger().info(
            f"Husky bridge ready: {input_topic} -> {output_topic}; "
            f"platform output {'available' if allow_output else 'inhibited'}; starts disabled"
        )

    def _simulated_time_active(self):
        return bool(
            self.has_parameter("use_sim_time")
            and self.get_parameter("use_sim_time").value
        )

    def _inhibit_if_simulated(self):
        if self.gate.allow_output and self._simulated_time_active():
            was_enabled = self.gate.disable("simulated_time")
            if was_enabled:
                self._queue_stop()
            self.gate.allow_output = False
            self.get_logger().warn("Simulated time enabled; platform output inhibited until restart")

    def _other_output_publisher_present(self):
        return any(
            endpoint.node_name != self.get_name()
            or endpoint.node_namespace != self.get_namespace()
            for endpoint in self.get_publishers_info_by_topic(self.output_topic)
        )

    def _drop_for_output_conflict(self):
        # A recording/replay session has no authority over the platform output.
        # Its normal manual teleop publisher must not mask LOCAL/REMOTE status.
        if not self.gate.allow_output:
            return False
        if self._other_output_publisher_present():
            was_enabled = self.gate.disable("output_topic_conflict")
            if was_enabled:
                self._queue_stop()
            return True
        return False

    def _queue_stop(self):
        if self.gate.allow_output:
            self.stop_until = time.monotonic() + self.stop_duration
            self.output_pub.publish(Twist())

    def _publish_state(self):
        state = "enabled" if self.gate.enabled else self.gate.reason
        if state != self.last_state:
            self.get_logger().info(f"Husky bridge state: {state}")
            self.last_state = state
        message = String()
        message.data = state
        self.state_pub.publish(message)

    def _publish_control_mode(self, now):
        mode = self.gate.mode(now)
        report = ControlModeReport()
        report.stamp = self.get_clock().now().to_msg()
        report.mode = {
            "autonomous": ControlModeReport.AUTONOMOUS,
            "manual": ControlModeReport.MANUAL,
            "no_command": ControlModeReport.NO_COMMAND,
            "not_ready": ControlModeReport.NOT_READY,
            "disengaged": ControlModeReport.DISENGAGED,
        }[mode]
        self.mode_pub.publish(report)
        operation = OperationModeState()
        operation.stamp = report.stamp
        operation.mode = {
            "autonomous": OperationModeState.AUTONOMOUS,
            "local": OperationModeState.LOCAL,
            "remote": OperationModeState.REMOTE,
            "stop": OperationModeState.STOP,
        }[self.gate.operation_mode(now)]
        operation.is_autoware_control_enabled = self.gate.enabled
        operation.is_in_transition = False
        operation.is_stop_mode_available = True
        operation.is_autonomous_mode_available = (
            self.gate._block_reason(now) is None
            and not self._other_output_publisher_present()
        )
        operation.is_local_mode_available = bool(self.local_manual_topics)
        operation.is_remote_mode_available = bool(self.remote_manual_topics)
        self.operation_mode_pub.publish(operation)
        if self.api_operation_mode_pub is not None:
            self.api_operation_mode_pub.publish(operation)

    def on_odometry(self, msg):
        if msg.child_frame_id != self.velocity_frame_id:
            self.get_logger().warn(
                f"Ignoring Husky odometry in {msg.child_frame_id!r}; "
                f"expected {self.velocity_frame_id!r}",
                throttle_duration_sec=5.0,
            )
            return
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return
        age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
        if age < -0.2 or age > self.max_odometry_age_s:
            return
        twist = msg.twist.twist
        if not all(isfinite(value) for value in (
            twist.linear.x, twist.linear.y, twist.angular.z
        )):
            return
        report = VelocityReport()
        report.header.stamp = msg.header.stamp
        report.header.frame_id = msg.child_frame_id
        report.longitudinal_velocity = float(twist.linear.x)
        report.lateral_velocity = float(twist.linear.y)
        report.heading_rate = float(twist.angular.z)
        self.gate.set_odometry(time.monotonic())
        self.velocity_pub.publish(report)

    def on_command(self, msg):
        was_enabled = self.gate.enabled
        if any(value != 0.0 for value in (
            msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y
        )):
            self.gate.command_at = None
            self.gate.disable("unsupported_twist_components")
        else:
            self.gate.set_command(msg.linear.x, msg.angular.z, time.monotonic())
        if was_enabled and not self.gate.enabled:
            self._queue_stop()

    def on_tracking(self, msg):
        was_enabled = self.gate.enabled
        self.gate.set_tracking(msg.data, time.monotonic())
        if was_enabled and not self.gate.enabled:
            self._queue_stop()
        self._publish_state()
        self._publish_control_mode(time.monotonic())

    def on_route_state(self, msg):
        if msg.state == RouteState.ARRIVED and self.gate.enabled:
            self.get_logger().info("Route arrived; switching Husky to Local mode")
            self._request_manual_mode("local")

    def on_estop(self, msg):
        was_enabled = self.gate.enabled
        self.gate.set_estop(bool(msg.data), time.monotonic())
        if was_enabled and not self.gate.enabled:
            self._queue_stop()

    def on_obstacle_stop(self, msg):
        self.obstacle_stop = bool(msg.data)
        self.obstacle_at = time.monotonic()
        if self.obstacle_stop and self.gate.enabled:
            self.output_pub.publish(Twist())

    def _obstacle_blocked(self, now):
        if not self.require_obstacle_status:
            return False
        return (
            self.obstacle_at is None
            or now - self.obstacle_at > self.obstacle_timeout
            or self.obstacle_stop
        )

    def on_manual(self, _msg, topic):
        was_enabled = self.gate.enabled
        source = "remote" if topic in self.remote_manual_topics else "local"
        self.gate.manual_takeover(time.monotonic(), source)
        if was_enabled:
            self._queue_stop()
        self._publish_control_mode(time.monotonic())

    def _request_enable(self):
        self._inhibit_if_simulated()
        output_conflict = self._drop_for_output_conflict()
        was_enabled = self.gate.enabled
        if output_conflict:
            success, reason = False, "output_topic_conflict"
        else:
            success, reason = self.gate.enable(time.monotonic())
        if success:
            self.stop_until = None
        if was_enabled and not self.gate.enabled:
            self._queue_stop()
        self._publish_state()
        self._publish_control_mode(time.monotonic())
        return success, reason

    def _request_disable(self, reason):
        if reason == "manual_request":
            source = self.gate.manual_source or (
                "remote" if self.remote_manual_topics else "local"
            )
            self._request_manual_mode(source)
            return
        was_enabled = self.gate.disable(reason)
        if was_enabled:
            self._queue_stop()
        self._publish_state()
        self._publish_control_mode(time.monotonic())

    def _request_manual_mode(self, source=None):
        was_enabled = self.gate.request_manual(source)
        if was_enabled:
            self._queue_stop()
        self._publish_state()
        self._publish_control_mode(time.monotonic())

    @staticmethod
    def _api_response(response, success, message):
        response.status.success = success
        response.status.code = 0 if success else ChangeOperationMode.Response.ERROR_NOT_AVAILABLE
        response.status.message = message
        return response

    def on_api_autonomous(self, _request, response):
        success, reason = self._request_enable()
        return self._api_response(response, success, reason)

    def on_api_stop(self, _request, response):
        self._request_disable("disabled")
        return self._api_response(response, True, "autonomous control disabled")

    def on_api_local(self, _request, response):
        self._request_manual_mode("local")
        return self._api_response(response, True, "local manual mode")

    def on_api_remote(self, _request, response):
        self._request_manual_mode("remote")
        return self._api_response(response, True, "remote manual mode")

    def on_api_disable(self, _request, response):
        self._request_disable("manual_request")
        return self._api_response(response, True, "autonomous control disabled")

    def on_enable_request(self, request, response):
        if request.data:
            response.success, response.message = self._request_enable()
        else:
            self._request_disable("disabled")
            response.success = True
            response.message = "disabled"
        return response

    def on_control_mode_request(self, request, response):
        if request.mode == ControlModeCommand.Request.AUTONOMOUS:
            response.success, reason = self._request_enable()
            if not response.success:
                self.get_logger().warn(f"Autonomous mode request rejected: {reason}")
        elif request.mode == ControlModeCommand.Request.MANUAL:
            self._request_disable("manual_request")
            response.success = True
        elif request.mode == ControlModeCommand.Request.NO_COMMAND:
            self._request_disable("disabled")
            response.success = True
        else:
            response.success = False
        return response

    def on_timer(self):
        self._inhibit_if_simulated()
        self._drop_for_output_conflict()
        now = time.monotonic()
        was_enabled = self.gate.enabled
        command = self.gate.output(now)
        if command is not None:
            message = Twist()
            if not self._obstacle_blocked(now):
                message.linear.x = command.linear
                message.angular.z = command.angular
            self.output_pub.publish(message)
        elif was_enabled:
            self._queue_stop()
        if self.stop_until is not None:
            if now < self.stop_until:
                self.output_pub.publish(Twist())
            else:
                self.stop_until = None
        self._publish_state()
        self._publish_control_mode(now)

    def stop_on_shutdown(self):
        if not (self.gate.allow_output and self.gate.ever_enabled):
            return
        self.gate.disable("shutdown")
        count = max(1, round(self.stop_duration / self.period))
        for _ in range(count):
            self.output_pub.publish(Twist())
            time.sleep(self.period)


def main():
    # Keep the ROS context alive long enough to transmit zero commands on SIGINT/SIGTERM.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    stopping = False

    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        node = HuskyTwistVehicleInterface()
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if node is not None:
            node.stop_on_shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

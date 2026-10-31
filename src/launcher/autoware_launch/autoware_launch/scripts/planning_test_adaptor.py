#!/usr/bin/env python3
"""Forward RViz goals and retry failed planning from the current localization pose."""

import time

import rclpy
from autoware_adapi_v1_msgs.srv import SetRoutePoints
from autoware_planning_msgs.msg import RouteState, Trajectory
from autoware_planning_msgs.srv import ClearRoute, SetWaypointRoute
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String


class PlanningTestAdaptor(Node):
    def __init__(self):
        super().__init__("planning_test_adaptor")
        self.route_client = self.create_client(SetWaypointRoute, "/planning/set_waypoint_route")
        self.clear_route_client = self.create_client(ClearRoute, "/planning/clear_route")
        self.auto_parameter_client = self.create_client(
            SetParameters, "/husky_twist_vehicle_interface/set_parameters"
        )
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.route_state = None
        self.request_in_flight = False
        self.manual_cancel_pending = False
        self.failure_goal = None
        self.recovery_route_ready = False
        self.recovery_route_cleared = False
        self.auto_route_disabled = False
        self.auto_restore_needed = False
        self.parameter_request_in_flight = False
        self.localization_at = None
        self.route_request_status_pub = self.create_publisher(
            String, "/planning/route_request_status", 10
        )
        self.rviz_trajectory_pub = self.create_publisher(
            Trajectory, "/planning/trajectory_rviz", 10
        )
        self.create_subscription(
            Trajectory, "/planning/trajectory", self.on_trajectory, 10
        )
        self.route_state_sub = self.create_subscription(
            RouteState, "/planning/route_state", self.on_route_state, state_qos
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, "/planning/mission_planning/goal", self.on_goal, 10
        )
        self.create_subscription(
            String, "/planning/manual_route_cancel", self.on_manual_route_cancel, 10
        )
        self.create_subscription(
            Odometry, "/localization/kinematic_state", self.on_localization,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0, self.on_retry_timer)

    def on_localization(self, msg):
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return
        age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
        if -0.2 <= age <= 1.0:
            self.localization_at = time.monotonic()

    def on_route_state(self, msg):
        self.route_state = msg.state
        if msg.state == RouteState.UNSET:
            self.manual_cancel_pending = False
            empty = Trajectory()
            empty.header.frame_id = "map"
            empty.header.stamp = self.get_clock().now().to_msg()
            self.rviz_trajectory_pub.publish(empty)

    def on_trajectory(self, msg):
        if self.route_state == RouteState.SET:
            self.rviz_trajectory_pub.publish(msg)

    def on_manual_route_cancel(self, _msg):
        if self.failure_goal is not None:
            return
        self.manual_cancel_pending = True
        self.try_clear_manual_route()

    def try_clear_manual_route(self):
        if not self.manual_cancel_pending or self.request_in_flight:
            return
        if not self.clear_route_client.service_is_ready():
            return
        self.request_in_flight = True
        future = self.clear_route_client.call_async(ClearRoute.Request())
        future.add_done_callback(self.on_manual_clear_result)

    def on_manual_clear_result(self, future):
        self.request_in_flight = False
        try:
            status = future.result().status
            if not status.success:
                raise RuntimeError(status.message)
        except Exception as exc:
            self.get_logger().error(f"Could not clear route after manual takeover: {exc}")
            return
        self.manual_cancel_pending = False
        self.get_logger().info("Manual takeover cleared the planning route")

    def on_goal(self, goal):
        if self.request_in_flight or self.manual_cancel_pending:
            self.get_logger().warn("Route request is still in progress; send the goal again")
            return
        if not self.route_client.service_is_ready():
            self.get_logger().warn("Route service is not ready; send the RViz goal again")
            return

        if self.failure_goal is not None:
            self.failure_goal = None
            self.recovery_route_ready = False
            self.recovery_route_cleared = False
            self.auto_restore_needed = self.auto_route_disabled or self.parameter_request_in_flight

        if self.route_state == RouteState.ARRIVED:
            if not self.clear_route_client.service_is_ready():
                self.get_logger().warn("Clear-route service is not ready; send the goal again")
                return
            self.route_request_status_pub.publish(String(data="pending"))
            self.request_in_flight = True
            future = self.clear_route_client.call_async(ClearRoute.Request())
            future.add_done_callback(lambda result: self.on_clear_result(result, goal))
            return

        self.route_request_status_pub.publish(String(data="pending"))
        self.send_route(goal)

    def on_clear_result(self, future, goal=None, recovery=False):
        try:
            status = future.result().status
        except Exception as exc:
            self.request_in_flight = False
            self.get_logger().error(f"Could not clear completed route: {exc}")
            return
        if not status.success:
            self.request_in_flight = False
            self.get_logger().error(f"Could not clear completed route: {status.message}")
            return
        if recovery:
            self.request_in_flight = False
            self.recovery_route_ready = False
            self.recovery_route_cleared = True
            self.try_set_auto_with_route(True)
            return
        self.get_logger().info("Completed route cleared; submitting the next goal")
        self.send_route(goal)

    def send_route(self, goal, retry=False):
        if not self.route_client.service_is_ready():
            self.request_in_flight = False
            self.get_logger().warn("Route service is not ready; send the RViz goal again")
            return
        request = SetWaypointRoute.Request()
        request.header = goal.header
        request.goal_pose = goal.pose
        request.allow_modification = True
        self.request_in_flight = True
        future = self.route_client.call_async(request)
        future.add_done_callback(
            lambda result: self.on_route_result(result, goal, retry=retry)
        )

    def on_route_result(self, future, goal, retry=False):
        self.request_in_flight = False
        try:
            status = future.result().status
        except Exception as exc:  # ROS service failure, not a planner response
            self.get_logger().error(f"Route service failed: {exc}")
            return
        if status.success:
            if retry:
                self.recovery_route_ready = True
                self.get_logger().info("Route recovered; clearing it before restoring auto mode")
            elif self.auto_restore_needed:
                self.try_set_auto_with_route(True)
            else:
                self.route_request_status_pub.publish(String(data="accepted"))
                self.get_logger().info("Lanelet route accepted")
        else:
            if status.code in (
                SetRoutePoints.Response.ERROR_PLANNER_FAILED,
                SetRoutePoints.Response.ERROR_REROUTE_FAILED,
            ):
                if self.failure_goal is None:
                    self.failure_goal = goal
                    self.manual_cancel_pending = False
                    self.auto_restore_needed = False
                    self.recovery_route_ready = False
                    self.recovery_route_cleared = False
                    self.route_request_status_pub.publish(String(data="rejected"))
                    self.try_set_auto_with_route(False)
            self.get_logger().error(f"Lanelet route rejected: {status.message}")

    def try_set_auto_with_route(self, enabled):
        if self.parameter_request_in_flight or not self.auto_parameter_client.service_is_ready():
            return
        request = SetParameters.Request()
        request.parameters = [
            ParameterMsg(
                name="enable_auto_with_route",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_BOOL, bool_value=enabled
                ),
            )
        ]
        self.parameter_request_in_flight = True
        future = self.auto_parameter_client.call_async(request)
        future.add_done_callback(
            lambda result: self.on_auto_parameter_result(result, enabled)
        )

    def on_auto_parameter_result(self, future, enabled):
        self.parameter_request_in_flight = False
        try:
            response = future.result()
            if len(response.results) != 1 or not response.results[0].successful:
                raise RuntimeError("vehicle interface rejected enable_auto_with_route")
        except Exception as exc:
            self.get_logger().error(f"Could not update enable_auto_with_route: {exc}")
            return
        self.auto_route_disabled = not enabled
        if enabled:
            if self.recovery_route_cleared:
                self.failure_goal = None
                self.recovery_route_cleared = False
                self.route_request_status_pub.publish(String(data="accepted"))
                self.get_logger().info("Recovered route cleared; robot status is ready")
            elif self.auto_restore_needed:
                self.auto_restore_needed = False
                self.route_request_status_pub.publish(String(data="accepted"))
        elif self.auto_restore_needed or self.recovery_route_cleared:
            self.try_set_auto_with_route(True)

    def on_retry_timer(self):
        if self.request_in_flight:
            return
        if self.manual_cancel_pending:
            self.try_clear_manual_route()
            return
        if self.recovery_route_cleared or (
            self.auto_restore_needed and self.failure_goal is None
        ):
            self.try_set_auto_with_route(True)
            return
        if self.failure_goal is None:
            return
        if not self.auto_route_disabled:
            self.try_set_auto_with_route(False)
            return
        if self.recovery_route_ready:
            if self.clear_route_client.service_is_ready():
                self.request_in_flight = True
                future = self.clear_route_client.call_async(ClearRoute.Request())
                future.add_done_callback(
                    lambda result: self.on_clear_result(result, recovery=True)
                )
            return
        if self.localization_at is None or time.monotonic() - self.localization_at > 1.0:
            self.get_logger().warn(
                "Waiting for fresh localization pose before retrying the route",
                throttle_duration_sec=5.0,
            )
            return
        if self.route_client.service_is_ready():
            self.send_route(self.failure_goal, retry=True)


def main():
    rclpy.init()
    node = PlanningTestAdaptor()
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

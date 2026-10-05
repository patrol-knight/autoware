#!/usr/bin/env python3
"""Forward RViz goals and replace completed routes while Husky is in manual mode."""

import rclpy
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_planning_msgs.msg import RouteState
from autoware_planning_msgs.srv import ClearRoute, SetWaypointRoute
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


class PlanningTestAdaptor(Node):
    def __init__(self):
        super().__init__("planning_test_adaptor")
        self.route_client = self.create_client(SetWaypointRoute, "/planning/set_waypoint_route")
        self.clear_route_client = self.create_client(ClearRoute, "/planning/clear_route")
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.route_state = None
        self.operation_mode_state = None
        self.request_in_flight = False
        self.route_state_sub = self.create_subscription(
            RouteState, "/planning/route_state", self.on_route_state, state_qos
        )
        self.operation_mode_sub = self.create_subscription(
            OperationModeState, "/system/operation_mode/state", self.on_operation_mode, state_qos
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, "/planning/mission_planning/goal", self.on_goal, 10
        )

    def on_route_state(self, msg):
        self.route_state = msg.state

    def on_operation_mode(self, msg):
        self.operation_mode_state = msg

    def is_manual(self):
        state = self.operation_mode_state
        return state is not None and (
            state.mode in (OperationModeState.LOCAL, OperationModeState.REMOTE)
            and not state.is_autoware_control_enabled
            and not state.is_in_transition
        )

    def on_goal(self, goal):
        if self.request_in_flight:
            self.get_logger().warn("Route request is still in progress; send the goal again")
            return
        if not self.route_client.service_is_ready():
            self.get_logger().warn("Route service is not ready; send the RViz goal again")
            return

        if self.route_state == RouteState.ARRIVED:
            if not self.is_manual():
                self.get_logger().warn(
                    "Completed route is still ARRIVED; switch Husky to manual mode "
                    "before setting another goal"
                )
                return
            if not self.clear_route_client.service_is_ready():
                self.get_logger().warn("Clear-route service is not ready; send the goal again")
                return
            self.request_in_flight = True
            future = self.clear_route_client.call_async(ClearRoute.Request())
            future.add_done_callback(lambda result: self.on_clear_result(result, goal))
            return

        self.send_route(goal)

    def on_clear_result(self, future, goal):
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
        if not self.is_manual():
            self.request_in_flight = False
            self.get_logger().warn("Route cleared, but Husky left manual mode; send the goal again")
            return

        self.get_logger().info("Completed route cleared; submitting the next goal")
        self.send_route(goal)

    def send_route(self, goal):
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
        future.add_done_callback(self.on_route_result)

    def on_route_result(self, future):
        self.request_in_flight = False
        try:
            status = future.result().status
        except Exception as exc:  # ROS service failure, not a planner response
            self.get_logger().error(f"Route service failed: {exc}")
            return
        if status.success:
            self.get_logger().info("Lanelet route accepted")
        else:
            self.get_logger().error(f"Lanelet route rejected: {status.message}")


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

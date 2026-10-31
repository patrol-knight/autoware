import sys
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace

import rclpy
from autoware_adapi_v1_msgs.srv import SetRoutePoints
from autoware_planning_msgs.msg import RouteState, Trajectory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from planning_test_adaptor import PlanningTestAdaptor


class FakeClient:
    def __init__(self, responses):
        self.responses = list(responses)
        self.requests = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(request)
        future = Future()
        future.set_result(self.responses.pop(0))
        return future


class FakePublisher:
    def __init__(self):
        self.values = []

    def publish(self, msg):
        self.values.append(msg.data)


class FakeTrajectoryPublisher:
    def __init__(self):
        self.values = []

    def publish(self, msg):
        self.values.append(msg)


def route_result(success, code=0):
    return SimpleNamespace(
        status=SimpleNamespace(success=success, code=code, message="test")
    )


def test_rviz_trajectory_clears_when_route_is_unset():
    rclpy.init()
    node = PlanningTestAdaptor()
    try:
        node.rviz_trajectory_pub = FakeTrajectoryPublisher()
        trajectory = Trajectory()
        trajectory.header.frame_id = "map"

        node.on_route_state(RouteState(state=RouteState.SET))
        node.on_trajectory(trajectory)
        assert node.rviz_trajectory_pub.values == [trajectory]

        node.on_route_state(RouteState(state=RouteState.UNSET))
        cleared = node.rviz_trajectory_pub.values[-1]
        assert cleared.header.frame_id == "map"
        assert not cleared.points
        node.on_trajectory(trajectory)
        assert node.rviz_trajectory_pub.values[-1] is cleared
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_failed_route_retries_until_success_then_clears_and_restores_auto():
    rclpy.init()
    node = PlanningTestAdaptor()
    try:
        node.route_client = FakeClient([
            route_result(False, SetRoutePoints.Response.ERROR_PLANNER_FAILED),
            route_result(False, SetRoutePoints.Response.ERROR_PLANNER_FAILED),
            route_result(True),
        ])
        node.clear_route_client = FakeClient([route_result(False), route_result(True)])
        node.auto_parameter_client = FakeClient([
            SimpleNamespace(results=[SimpleNamespace(successful=True)]),
            SimpleNamespace(results=[SimpleNamespace(successful=True)]),
        ])
        node.route_request_status_pub = FakePublisher()
        goal = PoseStamped()
        goal.header.frame_id = "map"

        node.on_goal(goal)
        assert node.route_request_status_pub.values == ["pending", "rejected"]
        assert node.auto_route_disabled
        assert len(node.route_client.requests) == 1
        assert node.route_client.requests[0].goal_pose == goal.pose
        assert len(node.auto_parameter_client.requests) == 1
        assert not node.auto_parameter_client.requests[0].parameters[0].value.bool_value

        node.on_manual_route_cancel(String(data="failure"))
        assert not node.manual_cancel_pending
        assert len(node.clear_route_client.requests) == 0

        localization = Odometry()
        localization.header.stamp = node.get_clock().now().to_msg()
        node.on_localization(localization)
        node.on_retry_timer()
        assert len(node.route_client.requests) == 2
        assert node.route_request_status_pub.values == ["pending", "rejected"]

        node.on_retry_timer()
        assert len(node.route_client.requests) == 3
        assert node.recovery_route_ready
        assert node.route_request_status_pub.values == ["pending", "rejected"]

        node.on_retry_timer()
        assert len(node.clear_route_client.requests) == 1
        assert node.route_request_status_pub.values == ["pending", "rejected"]

        node.on_retry_timer()
        assert len(node.clear_route_client.requests) == 2
        assert len(node.auto_parameter_client.requests) == 2
        assert node.auto_parameter_client.requests[1].parameters[0].value.bool_value
        assert node.route_request_status_pub.values == ["pending", "rejected", "accepted"]
        assert node.failure_goal is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_manual_takeover_retries_clear_route_until_success():
    rclpy.init()
    node = PlanningTestAdaptor()
    try:
        node.clear_route_client = FakeClient([route_result(False), route_result(True)])
        node.route_client = FakeClient([])
        node.on_route_state(RouteState(state=RouteState.SET))
        node.on_manual_route_cancel(String(data="navigate"))
        assert node.manual_cancel_pending
        assert len(node.clear_route_client.requests) == 1

        node.on_goal(PoseStamped())
        assert len(node.route_client.requests) == 0

        node.on_retry_timer()
        assert len(node.clear_route_client.requests) == 2
        assert not node.manual_cancel_pending
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_next_goal_after_arrival_clears_old_route_without_manual_mode():
    rclpy.init()
    node = PlanningTestAdaptor()
    try:
        node.clear_route_client = FakeClient([route_result(True)])
        node.route_client = FakeClient([route_result(True)])
        node.route_request_status_pub = FakePublisher()
        node.on_route_state(RouteState(state=RouteState.ARRIVED))
        goal = PoseStamped()
        goal.header.frame_id = "map"

        node.on_goal(goal)

        assert len(node.clear_route_client.requests) == 1
        assert len(node.route_client.requests) == 1
        assert node.route_client.requests[0].goal_pose == goal.pose
        assert node.route_request_status_pub.values == ["pending", "accepted"]
    finally:
        node.destroy_node()
        rclpy.shutdown()

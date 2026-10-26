import sys
import time
import unittest
from pathlib import Path

import rclpy
from autoware_planning_msgs.msg import RouteState, Trajectory, TrajectoryPoint
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from husky_twist_vehicle_interface import HuskyTwistVehicleInterface


class TestHuskyAutoWithRoute(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.bridge = HuskyTwistVehicleInterface(parameter_overrides=[
            Parameter("allow_platform_output", value=True),
            Parameter("enable_auto_with_route", value=True),
            Parameter("output_topic", value="/test/husky/auto_with_route/cmd_vel"),
        ])
        now = time.monotonic()
        self.bridge.gate.set_estop(False, now)
        self.bridge.gate.set_odometry(now)

    def tearDown(self):
        self.bridge.destroy_node()
        rclpy.shutdown()

    def set_route(self):
        route = RouteState(state=RouteState.SET)
        route.stamp = self.bridge.get_clock().now().to_msg()
        self.bridge.on_route_state(route)

    def send_trajectory_and_tracking(self, *, valid=True):
        trajectory = Trajectory()
        trajectory.header.frame_id = "map"
        trajectory.header.stamp = self.bridge.get_clock().now().to_msg()
        first = TrajectoryPoint()
        second = TrajectoryPoint()
        second.pose.position.x = 1.0 if valid else 0.0
        trajectory.points = [first, second]
        self.bridge.on_trajectory(trajectory)
        self.bridge.on_tracking(String(data="track"))
        command = Twist()
        command.linear.x = 0.1
        self.bridge.on_command(command)

    def test_valid_route_and_trajectory_enable_once(self):
        self.send_trajectory_and_tracking()
        self.bridge._maybe_auto_enable()
        self.assertFalse(self.bridge.gate.enabled)

        self.set_route()
        self.send_trajectory_and_tracking()
        self.bridge._maybe_auto_enable()
        self.assertTrue(self.bridge.gate.enabled)
        self.assertTrue(self.bridge.auto_started_for_route)

        self.bridge.on_route_state(RouteState(state=RouteState.ABORTED))
        self.assertFalse(self.bridge.gate.enabled)

        self.set_route()
        self.send_trajectory_and_tracking()
        self.bridge._maybe_auto_enable()
        self.assertTrue(self.bridge.gate.enabled)

        self.bridge.on_manual(Twist(), "/husky/joy_teleop/cmd_vel")
        self.assertFalse(self.bridge.gate.enabled)
        self.bridge.gate.manual_at = time.monotonic() - 2.0
        self.send_trajectory_and_tracking()
        self.bridge._maybe_auto_enable()
        self.assertFalse(self.bridge.gate.enabled)

    def test_invalid_trajectory_and_manual_takeover_do_not_enable(self):
        self.set_route()
        self.send_trajectory_and_tracking(valid=False)
        self.bridge._maybe_auto_enable()
        self.assertFalse(self.bridge.gate.enabled)

        self.bridge.on_manual(Twist(), "/husky/joy_teleop/cmd_vel")
        self.bridge.gate.manual_at = time.monotonic() - 2.0
        self.send_trajectory_and_tracking()
        self.bridge._maybe_auto_enable()
        self.assertFalse(self.bridge.gate.enabled)

    def test_disabled_platform_output_rejects_auto_enable(self):
        self.set_route()
        self.send_trajectory_and_tracking()
        self.bridge.gate.allow_output = False
        self.bridge._maybe_auto_enable()
        self.assertFalse(self.bridge.gate.enabled)

    def test_parameter_off_and_explicit_stop_prevent_auto_enable(self):
        self.set_route()
        self.send_trajectory_and_tracking()
        self.bridge.enable_auto_with_route = False
        self.bridge._maybe_auto_enable()
        self.assertFalse(self.bridge.gate.enabled)

        self.bridge.enable_auto_with_route = True
        self.bridge._request_disable("disabled")
        self.bridge._maybe_auto_enable()
        self.assertFalse(self.bridge.gate.enabled)

    def test_stale_trajectory_does_not_enable(self):
        self.set_route()
        self.send_trajectory_and_tracking()
        self.bridge.auto_trajectory.header.stamp.sec -= 2
        self.bridge._maybe_auto_enable()
        self.assertFalse(self.bridge.gate.enabled)

    def test_estop_during_route_blocks_later_auto_enable(self):
        self.set_route()
        self.bridge.on_estop(Bool(data=True))
        self.bridge.on_estop(Bool(data=False))
        self.send_trajectory_and_tracking()
        self.bridge._maybe_auto_enable()
        self.assertFalse(self.bridge.gate.enabled)

    def test_obstacle_status_must_be_clear_when_required(self):
        self.bridge.require_obstacle_status = True
        self.set_route()
        self.send_trajectory_and_tracking()
        self.bridge._maybe_auto_enable()
        self.assertFalse(self.bridge.gate.enabled)

        self.bridge.on_obstacle_stop(Bool(data=False))
        self.bridge._maybe_auto_enable()
        self.assertTrue(self.bridge.gate.enabled)

    def test_dynamic_parameter_disables_an_active_route(self):
        self.set_route()
        self.send_trajectory_and_tracking()
        self.bridge._maybe_auto_enable()
        self.assertTrue(self.bridge.gate.enabled)

        result = self.bridge.set_parameters([
            Parameter("enable_auto_with_route", value=False)
        ])
        self.assertTrue(result[0].successful)
        self.assertFalse(self.bridge.enable_auto_with_route)
        self.assertFalse(self.bridge.gate.enabled)
        self.assertTrue(self.bridge.auto_blocked_for_route)

        self.bridge.on_route_state(RouteState(state=RouteState.UNSET))
        self.set_route()
        self.send_trajectory_and_tracking()
        self.bridge._maybe_auto_enable()
        self.assertFalse(self.bridge.gate.enabled)

        result = self.bridge.set_parameters([
            Parameter("enable_auto_with_route", value=True)
        ])
        self.assertTrue(result[0].successful)
        self.assertTrue(self.bridge.auto_blocked_for_route)
        self.bridge._maybe_auto_enable()
        self.assertFalse(self.bridge.gate.enabled)

        self.bridge.on_route_state(RouteState(state=RouteState.UNSET))
        self.set_route()
        self.send_trajectory_and_tracking()
        self.bridge._maybe_auto_enable()
        self.assertTrue(self.bridge.gate.enabled)


if __name__ == "__main__":
    unittest.main()

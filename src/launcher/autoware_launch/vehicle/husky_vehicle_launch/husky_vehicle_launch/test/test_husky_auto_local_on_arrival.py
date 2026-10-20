import sys
import time
import unittest
from pathlib import Path

import rclpy
from autoware_planning_msgs.msg import RouteState
from rclpy.parameter import Parameter

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from husky_twist_vehicle_interface import HuskyTwistVehicleInterface


class TestHuskyAutoLocalOnArrival(unittest.TestCase):
    def test_arrival_switches_autonomous_to_local_once(self):
        rclpy.init()
        bridge = HuskyTwistVehicleInterface(parameter_overrides=[
            Parameter("allow_platform_output", value=True),
            Parameter("output_topic", value="/test/husky/arrival/cmd_vel"),
        ])
        try:
            now = time.monotonic()
            bridge.gate.set_estop(False, now)
            bridge.gate.set_odometry(now)
            bridge.gate.set_tracking("goal_reached", now)
            bridge.gate.set_command(0.0, 0.0, now)
            self.assertTrue(bridge._request_enable()[0])

            bridge.on_route_state(RouteState(state=RouteState.SET))
            self.assertTrue(bridge.gate.enabled)

            bridge.on_route_state(RouteState(state=RouteState.ARRIVED))
            self.assertFalse(bridge.gate.enabled)
            self.assertEqual(bridge.gate.operation_mode(time.monotonic()), "local")
            self.assertIsNotNone(bridge.stop_until)

            bridge._request_manual_mode("remote")
            bridge.on_route_state(RouteState(state=RouteState.ARRIVED))
            self.assertEqual(bridge.gate.operation_mode(time.monotonic()), "remote")
        finally:
            bridge.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()

import sys
import threading
import time
import unittest
from pathlib import Path

import rclpy
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from husky_twist_vehicle_interface import HuskyTwistVehicleInterface


class TestHuskyRvizModeApi(unittest.TestCase):
    def test_auto_and_manual_use_the_same_gate_with_test_output_topic(self):
        rclpy.init()
        bridge = HuskyTwistVehicleInterface(parameter_overrides=[
            Parameter("allow_platform_output", value=True),
            Parameter("rviz_operation_mode_api", value=True),
            Parameter("output_topic", value="/test/husky/cmd_vel"),
        ])
        try:
            now = time.monotonic()
            bridge.gate.set_estop(False, now)
            bridge.gate.set_odometry(now)
            bridge.gate.set_tracking("track", now)
            bridge.gate.set_command(0.1, 0.0, now)
            result = bridge.on_api_autonomous(
                ChangeOperationMode.Request(), ChangeOperationMode.Response()
            )
            self.assertTrue(result.status.success)
            self.assertTrue(bridge.gate.enabled)
            self.assertEqual(bridge.gate.operation_mode(time.monotonic()), "autonomous")

            result = bridge.on_api_disable(
                ChangeOperationMode.Request(), ChangeOperationMode.Response()
            )
            self.assertTrue(result.status.success)
            self.assertEqual(bridge.gate.operation_mode(time.monotonic()), "remote")
            self.assertFalse(bridge.gate.enabled)

            result = bridge.on_api_autonomous(
                ChangeOperationMode.Request(), ChangeOperationMode.Response()
            )
            self.assertTrue(result.status.success)

            result = bridge.on_api_remote(
                ChangeOperationMode.Request(), ChangeOperationMode.Response()
            )
            self.assertTrue(result.status.success)
            self.assertFalse(bridge.gate.enabled)
            self.assertEqual(bridge.gate.operation_mode(time.monotonic()), "remote")
        finally:
            bridge.destroy_node()
            rclpy.shutdown()

    def test_panel_services_use_the_inhibited_gate(self):
        rclpy.init()
        bridge = HuskyTwistVehicleInterface(parameter_overrides=[
            Parameter("allow_platform_output", value=False),
            Parameter("rviz_operation_mode_api", value=True),
        ])
        client_node = Node("husky_rviz_mode_api_test_client")
        executor = SingleThreadedExecutor()
        executor.add_node(bridge)
        executor.add_node(client_node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        states = []
        client_node.create_subscription(
            OperationModeState,
            "/api/operation_mode/state",
            lambda msg: states.append(msg),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        def call(name):
            client = client_node.create_client(
                ChangeOperationMode, f"/api/operation_mode/{name}"
            )
            self.assertTrue(client.wait_for_service(timeout_sec=2.0))
            future = client.call_async(ChangeOperationMode.Request())
            deadline = time.monotonic() + 2.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertTrue(future.done())
            return future.result()

        try:
            rejected = call("change_to_autonomous")
            self.assertFalse(rejected.status.success)
            self.assertEqual(rejected.status.message, "platform_output_disabled")
            self.assertFalse(bridge.gate.enabled)

            bridge.on_estop(Bool(data=False))
            self.assertTrue(call("change_to_remote").status.success)
            deadline = time.monotonic() + 2.0
            while not any(s.mode == OperationModeState.REMOTE for s in states):
                self.assertLess(time.monotonic(), deadline)
                time.sleep(0.01)
            self.assertTrue(call("change_to_stop").status.success)
            self.assertEqual(bridge.gate.operation_mode(time.monotonic()), "stop")
            self.assertFalse(bridge.gate.enabled)
        finally:
            executor.shutdown()
            thread.join(timeout=2.0)
            client_node.destroy_node()
            bridge.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()

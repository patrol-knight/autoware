import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from husky_twist_gate_core import TwistGate, Velocity


def gate(allow_output=True):
    return TwistGate(
        allow_output=allow_output,
        command_timeout=0.25,
        estop_timeout=2.0,
        odometry_timeout=0.7,
        tracking_timeout=0.5,
        manual_hold=1.0,
        max_linear=0.3,
        max_angular=0.5,
    )


class TestTwistGate(unittest.TestCase):
    def test_requires_explicit_enable_and_fresh_estop_and_command(self):
        controller = gate()
        controller.set_command(0.2, 0.1, 10.0)
        self.assertIsNone(controller.output(10.0))
        self.assertEqual(controller.enable(10.0), (False, "estop_status_missing_or_stale"))
        controller.set_estop(False, 10.0)
        controller.set_odometry(10.0)
        controller.set_tracking("track", 10.0)
        self.assertEqual(controller.enable(10.1), (True, "enabled"))
        self.assertEqual(controller.output(10.1), Velocity(0.2, 0.1))
        self.assertIsNone(controller.output(10.3))
        self.assertEqual(controller.reason, "command_missing_or_stale")
        controller.set_command(0.1, 0.0, 10.31)
        self.assertIsNone(controller.output(10.31))
        self.assertEqual(controller.enable(12.1), (False, "estop_status_missing_or_stale"))

    def test_manual_and_estop_latch_until_reenabled(self):
        controller = gate()
        controller.set_estop(False, 0.0)
        controller.set_odometry(0.0)
        controller.set_tracking("track", 0.0)
        controller.set_command(0.2, 0.0, 0.0)
        self.assertTrue(controller.enable(0.0)[0])
        controller.manual_takeover(0.1)
        self.assertIsNone(controller.output(0.1))
        self.assertFalse(controller.enable(0.5)[0])
        controller.set_command(0.2, 0.0, 1.2)
        controller.set_odometry(1.2)
        controller.set_tracking("track", 1.2)
        self.assertTrue(controller.enable(1.2)[0])
        controller.set_estop(True, 1.3)
        self.assertIsNone(controller.output(1.3))
        controller.set_estop(False, 1.4)
        controller.set_odometry(1.4)
        controller.set_command(0.2, 0.0, 1.4)
        self.assertIsNone(controller.output(1.4))
        self.assertTrue(controller.enable(1.4)[0])

    def test_output_switch_and_invalid_or_overspeed_command(self):
        controller = gate(allow_output=False)
        controller.set_estop(False, 0.0)
        controller.set_odometry(0.0)
        controller.set_tracking("track", 0.0)
        controller.set_command(0.2, 0.0, 0.0)
        self.assertEqual(controller.enable(0.0), (False, "platform_output_disabled"))
        controller = gate()
        controller.set_estop(False, 0.0)
        controller.set_odometry(0.0)
        controller.set_tracking("track", 0.0)
        controller.set_command(0.2, 0.0, 0.0)
        self.assertTrue(controller.enable(0.0)[0])
        controller.set_command(float("nan"), 0.0, 0.1)
        self.assertIsNone(controller.output(0.1))
        self.assertEqual(controller.reason, "invalid_command")
        controller.set_command(1.0, 0.0, 0.2)
        self.assertEqual(controller.enable(0.2), (False, "command_missing_or_stale"))
        self.assertEqual(controller.reason, "command_missing_or_stale")

    def test_feedback_is_required_and_modes_follow_actual_gate_state(self):
        controller = gate()
        controller.set_estop(False, 0.0)
        controller.set_command(0.2, 0.0, 0.0)
        self.assertEqual(controller.enable(0.0), (False, "odometry_missing_or_stale"))
        self.assertEqual(controller.mode(0.0), "not_ready")
        controller.set_odometry(0.0)
        controller.set_tracking("track", 0.0)
        self.assertTrue(controller.enable(0.0)[0])
        self.assertEqual(controller.mode(0.0), "autonomous")
        controller.manual_takeover(0.1)
        self.assertEqual(controller.mode(0.1), "manual")
        controller.disable()
        self.assertEqual(controller.mode(0.1), "disengaged")
        controller.disable("manual_request")
        self.assertEqual(controller.mode(0.1), "manual")
        controller.set_estop(True, 0.2)
        self.assertEqual(controller.mode(0.2), "not_ready")
        controller.disable()
        self.assertEqual(controller.mode(0.2), "not_ready")
        controller.set_estop(False, 0.3)
        controller.set_command(0.2, 0.0, 1.2)
        self.assertEqual(controller.enable(1.2), (False, "odometry_missing_or_stale"))

    def test_operation_mode_tracks_local_remote_autonomy_and_faults(self):
        controller = gate()
        self.assertEqual(controller.operation_mode(0.0), "stop")
        controller.set_estop(False, 0.0)
        controller.set_odometry(0.0)
        controller.set_tracking("track", 0.0)
        controller.set_command(0.2, 0.0, 0.0)
        self.assertTrue(controller.enable(0.0)[0])
        self.assertEqual(controller.operation_mode(0.0), "autonomous")
        controller.manual_takeover(0.1, "remote")
        self.assertEqual(controller.operation_mode(0.1), "remote")
        controller.manual_takeover(0.2, "local")
        self.assertEqual(controller.operation_mode(0.2), "local")
        controller.set_tracking("too_far_from_path", 0.21)
        self.assertEqual(controller.operation_mode(0.21), "local")
        controller.set_estop(True, 0.3)
        self.assertEqual(controller.operation_mode(0.3), "stop")
        controller.set_estop(False, 0.4)
        controller.request_manual()
        self.assertEqual(controller.operation_mode(0.4), "local")
        controller.disable("output_topic_conflict")
        self.assertEqual(controller.mode(0.4), "not_ready")
        self.assertEqual(controller.operation_mode(0.4), "stop")
        controller.disable("disabled")
        self.assertEqual(controller.operation_mode(0.4), "stop")

    def test_manual_source_must_be_known(self):
        controller = gate()
        with self.assertRaises(ValueError):
            controller.manual_takeover(0.0, "unknown")
        with self.assertRaises(ValueError):
            controller.request_manual("unknown")

    def test_rviz_manual_mode_selection_preserves_source(self):
        controller = gate()
        controller.set_estop(False, 10.0)
        controller.request_manual("remote")
        self.assertEqual(controller.operation_mode(10.0), "remote")
        controller.request_manual()
        self.assertEqual(controller.operation_mode(10.0), "remote")
        controller.request_manual("local")
        self.assertEqual(controller.operation_mode(10.0), "local")

    def test_manual_status_survives_disabled_output_and_command_updates(self):
        controller = gate(allow_output=False)
        controller.set_estop(False, 0.0)
        controller.manual_takeover(0.1, "local")
        self.assertEqual(controller.mode(0.1), "manual")
        self.assertEqual(controller.operation_mode(0.1), "local")
        controller.set_command(1.0, 0.0, 0.2)
        self.assertEqual(controller.reason, "command_out_of_range")
        self.assertEqual(controller.mode(0.2), "manual")
        self.assertEqual(controller.operation_mode(0.2), "local")
        self.assertEqual(controller.mode(1.2), "no_command")
        self.assertEqual(controller.operation_mode(1.2), "stop")
        controller.set_estop(True, 0.3)
        self.assertEqual(controller.operation_mode(0.3), "stop")
        controller.disable("disabled")
        self.assertEqual(controller.mode(0.3), "no_command")

    def test_tracking_fault_and_timeout_disengage_autonomy(self):
        controller = gate()
        controller.set_estop(False, 0.0)
        controller.set_odometry(0.0)
        controller.set_command(0.2, 0.0, 0.0)
        self.assertEqual(controller.enable(0.0), (False, "tracking_missing_or_stale"))
        controller.set_tracking("track", 0.0)
        self.assertTrue(controller.enable(0.0)[0])
        controller.set_tracking("too_far_from_path", 0.1)
        self.assertEqual(controller.operation_mode(0.1), "stop")
        self.assertEqual(controller.mode(0.1), "not_ready")
        controller.set_tracking("track", 0.2)
        self.assertFalse(controller.enabled)
        controller.set_estop(False, 0.2)
        controller.set_odometry(0.2)
        controller.set_command(0.2, 0.0, 0.2)
        self.assertTrue(controller.enable(0.2)[0])
        controller.set_estop(False, 0.6)
        controller.set_odometry(0.6)
        controller.set_command(0.2, 0.0, 0.6)
        self.assertIsNone(controller.output(0.8))
        self.assertEqual(controller.reason, "tracking_missing_or_stale")


if __name__ == "__main__":
    unittest.main()

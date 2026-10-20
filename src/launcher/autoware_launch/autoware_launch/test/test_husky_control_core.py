"""Behavior checks for the Husky trajectory follower geometry."""

import sys
import unittest
from math import pi
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from husky_control_core import (
    Limits, Pose2D, Result, Waypoint, apply_velocity_feedback, calculate_command,
)


class TestHuskyControlCore(unittest.TestCase):
    def setUp(self):
        self.limits = Limits()

    def test_straight_and_turn_signs(self):
        straight = [Waypoint(0, 0, 0, 0.2), Waypoint(2, 0, 0, 0.2)]
        result = calculate_command(straight, Pose2D(0, 0, 0), 0, 0, 0.1, self.limits)
        self.assertEqual(result.state, "track")
        self.assertGreater(result.linear, 0)
        self.assertAlmostEqual(result.angular, 0)

        left = [Waypoint(0, 0, 0, 0.2), Waypoint(2, 1, 0.4, 0.2)]
        right = [Waypoint(0, 0, 0, 0.2), Waypoint(2, -1, -0.4, 0.2)]
        left_cmd = calculate_command(left, Pose2D(0, 0, 0), 0, 0, 0.1, self.limits)
        right_cmd = calculate_command(right, Pose2D(0, 0, 0), 0, 0, 0.1, self.limits)
        self.assertGreater(left_cmd.angular, 0)
        self.assertLess(right_cmd.angular, 0)

    def test_align_path_and_terminal_heading_without_translation(self):
        straight = [Waypoint(0, 0, 0, 0.2), Waypoint(2, 0, 0, 0.2)]
        result = calculate_command(straight, Pose2D(0, 0, pi / 2), 0, 0, 0.1, self.limits)
        self.assertEqual(result.state, "align_path")
        self.assertEqual(result.linear, 0)
        self.assertLess(result.angular, 0)

        goal = [Waypoint(0, 0, 0, 0.2), Waypoint(1, 0, pi / 2, 0)]
        result = calculate_command(goal, Pose2D(1, 0, 0), 0, 0, 0.1, self.limits)
        self.assertEqual(result.state, "align_goal")
        self.assertEqual(result.linear, 0)
        self.assertGreater(result.angular, 0)

        result = calculate_command(goal, Pose2D(1, 0, pi / 2), 0, 0.2, 0.1, self.limits)
        self.assertEqual(result.state, "goal_reached")
        self.assertEqual((result.linear, result.angular), (0, 0))

    def test_stop_speed_limit_and_reverse_rejection(self):
        goal = [Waypoint(0, 0, 0, 0.2), Waypoint(1, 0, 0, 0)]
        result = calculate_command(goal, Pose2D(0.99, 0, 0), 0.2, 0.1, 0.1, self.limits)
        self.assertEqual(result.state, "goal_reached")
        self.assertEqual((result.linear, result.angular), (0, 0))

        fast = [Waypoint(0, 0, 0, 5), Waypoint(2, 0, 0, 5)]
        result = calculate_command(fast, Pose2D(0, 0, 0), 0, 0, 0.1, self.limits)
        self.assertLessEqual(result.linear, self.limits.max_linear_accel * 0.1 + 1e-9)

        reverse = [Waypoint(0, 0, 0, -0.2), Waypoint(2, 0, 0, -0.2)]
        result = calculate_command(reverse, Pose2D(0, 0, 0), 0, 0, 0.1, self.limits)
        self.assertEqual(result.state, "reverse_unsupported")
        self.assertEqual((result.linear, result.angular), (0, 0))

        far = calculate_command(fast, Pose2D(0, 2, 0), 0, 0, 0.1, self.limits)
        self.assertEqual(far.state, "too_far_from_path")
        self.assertEqual((far.linear, far.angular), (0, 0))

    def test_zero_speed_start_can_move_but_intermediate_stop_holds(self):
        starting = [
            Waypoint(0, 0, 0, 0),
            Waypoint(0.5, 0, 0, 0.2),
            Waypoint(2, 0, 0, 0.2),
        ]
        result = calculate_command(starting, Pose2D(0, 0, 0), 0, 0, 0.1, self.limits)
        self.assertEqual(result.state, "track")
        self.assertGreater(result.linear, 0)

        stopping = [
            Waypoint(0, 0, 0, 0.2),
            Waypoint(1, 0, 0, 0),
            Waypoint(2, 0, 0, 0.2),
        ]
        result = calculate_command(stopping, Pose2D(1, 0, 0), 0.1, 0, 0.1, self.limits)
        self.assertEqual(result.state, "planned_stop")
        self.assertEqual((result.linear, result.angular), (0, 0))

    def test_body_velocity_feedback_respects_direction_limits_and_stop(self):
        target = Result(0.2, 0.2, "track")
        slower = apply_velocity_feedback(
            target, 0.1, 0.1, 0.2, 0.2, 0.1, self.limits, 0.5, 0.5
        )
        self.assertGreater(slower.linear, target.linear)
        self.assertGreater(slower.angular, target.angular)
        faster = apply_velocity_feedback(
            target, 0.3, 0.3, 0.2, 0.2, 0.1, self.limits, 0.5, 0.5
        )
        self.assertLess(faster.linear, target.linear)
        self.assertLess(faster.angular, target.angular)
        stopped = apply_velocity_feedback(
            Result(0.0, 0.0, "planned_stop"), 0.2, 0.1,
            0.2, 0.1, 0.1, self.limits, 0.5, 0.5
        )
        self.assertEqual((stopped.linear, stopped.angular), (0.0, 0.0))
        invalid = apply_velocity_feedback(
            target, float("nan"), 0.0, 0.2, 0.2, 0.1,
            self.limits, 0.5, 0.5
        )
        self.assertEqual((invalid.linear, invalid.angular), (0.0, 0.0))
        self.assertEqual(invalid.state, "invalid_velocity_feedback")

    def test_feedback_does_not_lock_turn_opposite_to_path(self):
        # A previous saturated right turn must unwind when the path moves left.
        path = [Waypoint(0.0, 0.3, 0.0, 0.3), Waypoint(5.0, 0.3, 0.0, 0.3)]
        pose = Pose2D(1.0, 0.0, 0.0)
        linear, angular = 0.3, -self.limits.max_angular_speed
        for _ in range(30):
            target = calculate_command(path, pose, linear, angular, 0.05, self.limits)
            result = apply_velocity_feedback(
                target, 0.2, 0.0, linear, angular, 0.05,
                self.limits, 0.5, 0.5,
            )
            self.assertGreaterEqual(result.angular, angular)
            linear, angular = result.linear, result.angular
        self.assertGreater(angular, 0.0)


if __name__ == "__main__":
    unittest.main()

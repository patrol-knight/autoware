import sys
from pathlib import Path

import rclpy
from autoware_planning_msgs.msg import RouteState, Trajectory

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from husky_skid_steer_trajectory_follower import HuskySkidSteerTrajectoryFollower


class FakePublisher:
    def __init__(self):
        self.values = []

    def publish(self, msg):
        self.values.append(msg)


def test_clear_route_discards_trajectory_and_publishes_stop():
    rclpy.init()
    node = HuskySkidSteerTrajectoryFollower()
    try:
        node.command_pub = FakePublisher()
        node.state_pub = FakePublisher()
        trajectory = Trajectory()
        node.on_route_state(RouteState(state=RouteState.SET))
        node.on_trajectory(trajectory)
        assert node.trajectory is trajectory

        node.on_route_state(RouteState(state=RouteState.UNSET))
        assert node.trajectory is None
        assert node.command_pub.values[-1].linear.x == 0.0
        assert node.command_pub.values[-1].angular.z == 0.0
        node.on_trajectory(trajectory)
        assert node.trajectory is None
    finally:
        node.destroy_node()
        rclpy.shutdown()

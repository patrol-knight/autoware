#!/usr/bin/env python3
"""Supply manual mode, forward RViz goals, and visualize the planned trajectory."""

import rclpy
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_planning_msgs.msg import Trajectory
from autoware_planning_msgs.srv import SetWaypointRoute
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker


class PlanningTestAdaptor(Node):
    def __init__(self):
        super().__init__("planning_test_adaptor")
        self.operation_mode_pub = self.create_publisher(
            OperationModeState,
            "/system/operation_mode/state",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.route_client = self.create_client(SetWaypointRoute, "/planning/set_waypoint_route")
        self.goal_sub = self.create_subscription(
            PoseStamped, "/planning/mission_planning/goal", self.on_goal, 10
        )
        self.trajectory_marker_pub = self.create_publisher(
            Marker,
            "/planning/trajectory_marker",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.trajectory_sub = self.create_subscription(
            Trajectory, "/planning/trajectory", self.on_trajectory, 10
        )
        self.timer = self.create_timer(1.0, self.publish_manual_mode)
        self.publish_manual_mode()

    def publish_manual_mode(self):
        state = OperationModeState()
        state.stamp = self.get_clock().now().to_msg()
        state.mode = OperationModeState.LOCAL
        state.is_autoware_control_enabled = False
        state.is_local_mode_available = True
        self.operation_mode_pub.publish(state)

    def on_goal(self, goal):
        if not self.route_client.service_is_ready():
            self.get_logger().warn("Route service is not ready; send the RViz goal again")
            return
        request = SetWaypointRoute.Request()
        request.header = goal.header
        request.goal_pose = goal.pose
        request.allow_modification = True
        future = self.route_client.call_async(request)
        future.add_done_callback(self.on_route_result)

    def on_route_result(self, future):
        try:
            status = future.result().status
        except Exception as exc:  # ROS service failure, not a planner response
            self.get_logger().error(f"Route service failed: {exc}")
            return
        if status.success:
            self.get_logger().info("Lanelet route accepted")
        else:
            self.get_logger().error(f"Lanelet route rejected: {status.message}")

    def on_trajectory(self, trajectory):
        marker = Marker()
        marker.header = trajectory.header
        marker.ns = "husky_planned_trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD if trajectory.points else Marker.DELETE
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.18
        marker.color.g = 1.0
        marker.color.a = 1.0
        # Keep the planned data unchanged; lift only its RViz marker above the lanelet surface.
        marker.points = [
            Point(
                x=point.pose.position.x,
                y=point.pose.position.y,
                z=point.pose.position.z + 0.35,
            )
            for point in trajectory.points
        ]
        self.trajectory_marker_pub.publish(marker)


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

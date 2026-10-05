#!/usr/bin/env python3
"""Body-velocity trajectory follower for a skid-steer Husky.

The output is an internal Twist topic. A separate, explicitly armed vehicle
interface decides whether this command reaches the Clearpath platform.
"""

from math import atan2, cos, isfinite, sin

import rclpy
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from autoware_vehicle_msgs.msg import VelocityReport
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String

from husky_control_core import (
    Limits, Pose2D, Waypoint, apply_velocity_feedback, calculate_command, wrap_angle,
)


def yaw_from_quaternion(quaternion) -> float:
    return atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class HuskySkidSteerTrajectoryFollower(Node):
    def __init__(self):
        super().__init__("husky_skid_steer_trajectory_follower")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("max_input_age_s", 0.7)
        self.declare_parameter("lookahead_distance", 0.6)
        self.declare_parameter("max_path_distance", 1.0)
        self.declare_parameter("max_linear_speed", 0.3)
        self.declare_parameter("max_angular_speed", 0.5)
        self.declare_parameter("max_linear_accel", 0.3)
        self.declare_parameter("max_linear_decel", 0.3)
        self.declare_parameter("max_angular_accel", 0.5)
        self.declare_parameter("heading_gain", 1.5)
        self.declare_parameter("rotate_to_path_threshold", 0.7)
        self.declare_parameter("goal_position_tolerance", 0.15)
        self.declare_parameter("goal_yaw_tolerance", 0.10)
        self.declare_parameter("stop_speed_threshold", 0.03)
        self.declare_parameter("prediction_horizon_s", 2.0)
        self.declare_parameter("prediction_step_s", 0.1)
        self.declare_parameter("linear_velocity_feedback_gain", 0.5)
        self.declare_parameter("angular_velocity_feedback_gain", 0.5)

        self.limits = Limits(**{
            name: self.get_parameter(name).value
            for name in Limits.__dataclass_fields__
        })
        self.max_input_age_s = float(self.get_parameter("max_input_age_s").value)
        self.prediction_horizon_s = float(self.get_parameter("prediction_horizon_s").value)
        self.prediction_step_s = float(self.get_parameter("prediction_step_s").value)
        self.linear_velocity_gain = float(
            self.get_parameter("linear_velocity_feedback_gain").value
        )
        self.angular_velocity_gain = float(
            self.get_parameter("angular_velocity_feedback_gain").value
        )
        rate = float(self.get_parameter("control_rate_hz").value)
        if rate <= 0.0 or self.max_input_age_s <= 0.0 or self.prediction_step_s <= 0.0:
            raise ValueError("Control rate, input age and prediction step must be positive")
        if not all(isfinite(value) and value >= 0.0 for value in (
            self.linear_velocity_gain, self.angular_velocity_gain
        )):
            raise ValueError("Velocity feedback gains must be finite and nonnegative")
        if any(value <= 0.0 for value in (
            self.limits.lookahead_distance,
            self.limits.max_path_distance,
            self.limits.max_linear_speed,
            self.limits.max_angular_speed,
            self.limits.max_linear_accel,
            self.limits.max_linear_decel,
            self.limits.max_angular_accel,
        )):
            raise ValueError("Lookahead and speed/acceleration limits must be positive")

        self.trajectory = None
        self.odometry = None
        self.velocity_report = None
        self.previous_linear = 0.0
        self.previous_angular = 0.0
        self.last_tick = None
        self.last_state = None

        self.command_pub = self.create_publisher(Twist, "/control/husky/cmd_vel", 10)
        self.prediction_pub = self.create_publisher(
            Trajectory, "/control/trajectory_follower/lateral/predicted_trajectory", 10
        )
        self.state_pub = self.create_publisher(String, "/control/husky/tracking_state", 10)
        self.trajectory_sub = self.create_subscription(
            Trajectory, "/planning/trajectory", self.on_trajectory, 10
        )
        self.odometry_sub = self.create_subscription(
            Odometry, "/localization/kinematic_state", self.on_odometry, qos_profile_sensor_data
        )
        self.velocity_sub = self.create_subscription(
            VelocityReport, "/vehicle/status/velocity_status", self.on_velocity, 10
        )
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info("Husky follower ready: /control/husky/cmd_vel")

    def on_trajectory(self, msg):
        self.trajectory = msg

    def on_odometry(self, msg):
        self.odometry = msg

    def on_velocity(self, msg):
        self.velocity_report = msg

    def is_fresh(self, stamp, now):
        if stamp.sec == 0 and stamp.nanosec == 0:
            return False
        age = (now - Time.from_msg(stamp)).nanoseconds / 1e9
        return -0.2 <= age <= self.max_input_age_s

    def publish_stop(self, state):
        self.previous_linear = 0.0
        self.previous_angular = 0.0
        self.command_pub.publish(Twist())
        self.publish_state(state)

    def publish_state(self, state):
        if state != self.last_state:
            self.get_logger().info(f"Husky tracker state: {state}")
            self.last_state = state
        message = String()
        message.data = state
        self.state_pub.publish(message)

    def publish_prediction(self, pose, linear, angular, frame_id, stamp):
        prediction = Trajectory()
        prediction.header.frame_id = frame_id
        prediction.header.stamp = stamp
        x, y, yaw = pose.x, pose.y, pose.yaw
        count = int(self.prediction_horizon_s / self.prediction_step_s)
        for index in range(count + 1):
            point = TrajectoryPoint()
            point.pose.position.x = x
            point.pose.position.y = y
            point.pose.orientation.z = sin(yaw / 2.0)
            point.pose.orientation.w = cos(yaw / 2.0)
            point.longitudinal_velocity_mps = float(linear)
            point.heading_rate_rps = float(angular)
            elapsed_ns = round(index * self.prediction_step_s * 1e9)
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(elapsed_ns, 1_000_000_000)
            prediction.points.append(point)
            if abs(angular) < 1e-6:
                x += linear * cos(yaw) * self.prediction_step_s
                y += linear * sin(yaw) * self.prediction_step_s
            else:
                next_yaw = yaw + angular * self.prediction_step_s
                x += linear * (sin(next_yaw) - sin(yaw)) / angular
                y -= linear * (cos(next_yaw) - cos(yaw)) / angular
                yaw = wrap_angle(next_yaw)
        self.prediction_pub.publish(prediction)

    def on_timer(self):
        now = self.get_clock().now()
        if self.last_tick is None:
            dt = self.timer.timer_period_ns / 1e9
        else:
            dt = (now - self.last_tick).nanoseconds / 1e9
            if dt <= 0.0 or dt > 0.5:
                self.previous_linear = 0.0
                self.previous_angular = 0.0
                dt = self.timer.timer_period_ns / 1e9
        self.last_tick = now

        if self.trajectory is None or self.odometry is None or self.velocity_report is None:
            self.publish_stop("waiting_for_inputs")
            return
        if not self.is_fresh(self.trajectory.header.stamp, now) or not self.is_fresh(
            self.odometry.header.stamp, now
        ) or not self.is_fresh(
            self.velocity_report.header.stamp, now
        ):
            self.publish_stop("stale_input")
            return
        if not self.trajectory.header.frame_id or (
            self.trajectory.header.frame_id != self.odometry.header.frame_id
        ):
            self.publish_stop("frame_mismatch")
            return
        if (
            not self.odometry.child_frame_id
            or self.velocity_report.header.frame_id != self.odometry.child_frame_id
        ):
            self.publish_stop("velocity_frame_mismatch")
            return

        pose_msg = self.odometry.pose.pose
        pose = Pose2D(
            pose_msg.position.x,
            pose_msg.position.y,
            yaw_from_quaternion(pose_msg.orientation),
        )
        points = [
            Waypoint(
                point.pose.position.x,
                point.pose.position.y,
                yaw_from_quaternion(point.pose.orientation),
                point.longitudinal_velocity_mps,
            )
            for point in self.trajectory.points
        ]
        result = calculate_command(
            points, pose, self.previous_linear, self.previous_angular, dt, self.limits
        )
        result = apply_velocity_feedback(
            result,
            self.velocity_report.longitudinal_velocity,
            self.velocity_report.heading_rate,
            self.previous_linear,
            self.previous_angular,
            dt,
            self.limits,
            self.linear_velocity_gain,
            self.angular_velocity_gain,
        )
        if not all(isfinite(value) for value in (result.linear, result.angular)):
            self.publish_stop("invalid_command")
            return
        command = Twist()
        command.linear.x = result.linear
        command.angular.z = result.angular
        self.command_pub.publish(command)
        self.previous_linear = result.linear
        self.previous_angular = result.angular
        self.publish_state(result.state)
        self.publish_prediction(
            pose, result.linear, result.angular, self.trajectory.header.frame_id, now.to_msg()
        )


def main():
    rclpy.init()
    node = HuskySkidSteerTrajectoryFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop("shutdown")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

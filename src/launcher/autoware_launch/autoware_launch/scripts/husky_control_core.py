"""Planar path tracking for the Husky skid-steer controller.

This module deliberately has no ROS dependencies so the geometry and stop
behavior can be checked without starting a vehicle or a ROS graph.
"""

from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, sin, sqrt


def wrap_angle(angle: float) -> float:
    return atan2(sin(angle), cos(angle))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: float
    speed: float


@dataclass(frozen=True)
class Limits:
    lookahead_distance: float = 0.6
    max_path_distance: float = 1.0
    max_linear_speed: float = 0.3
    max_angular_speed: float = 0.5
    max_linear_accel: float = 0.3
    max_linear_decel: float = 0.3
    max_angular_accel: float = 0.5
    heading_gain: float = 1.5
    rotate_to_path_threshold: float = 0.7
    goal_position_tolerance: float = 0.15
    goal_yaw_tolerance: float = 0.10
    stop_speed_threshold: float = 0.03


@dataclass(frozen=True)
class Result:
    linear: float
    angular: float
    state: str
    lateral_error: float = 0.0
    heading_error: float = 0.0


def _rate_limit(target: float, previous: float, maximum_delta: float) -> float:
    return clamp(target, previous - maximum_delta, previous + maximum_delta)


def apply_velocity_feedback(
    target: Result,
    measured_linear: float,
    measured_angular: float,
    previous_linear: float,
    previous_angular: float,
    dt: float,
    limits: Limits,
    linear_gain: float,
    angular_gain: float,
) -> Result:
    """Add bounded body-velocity feedback to a geometric tracking command."""
    if not all(isfinite(value) for value in (
        measured_linear, measured_angular, previous_linear, previous_angular,
        dt, linear_gain, angular_gain,
    )) or dt <= 0.0 or min(linear_gain, angular_gain) < 0.0:
        return Result(0.0, 0.0, "invalid_velocity_feedback")
    if target.state not in ("track", "align_path", "align_goal"):
        return target

    linear = 0.0
    if target.linear > 0.0:
        wanted_linear = clamp(
            target.linear + linear_gain * (target.linear - measured_linear),
            0.0, limits.max_linear_speed,
        )
        delta = (
            limits.max_linear_accel if wanted_linear >= previous_linear
            else limits.max_linear_decel
        ) * dt
        linear = clamp(
            _rate_limit(wanted_linear, previous_linear, delta),
            0.0, limits.max_linear_speed,
        )

    angular = 0.0
    if target.angular != 0.0:
        wanted_angular = target.angular + angular_gain * (
            target.angular - measured_angular
        )
        if target.angular > 0.0:
            wanted_angular = clamp(wanted_angular, 0.0, limits.max_angular_speed)
        else:
            wanted_angular = clamp(wanted_angular, -limits.max_angular_speed, 0.0)
        angular = clamp(
            _rate_limit(
                wanted_angular, previous_angular, limits.max_angular_accel * dt
            ),
            -limits.max_angular_speed, limits.max_angular_speed,
        )
    return Result(linear, angular, target.state, target.lateral_error, target.heading_error)


def _cumulative_distances(points: list[Waypoint]) -> list[float]:
    cumulative = [0.0]
    for a, b in zip(points[:-1], points[1:]):
        cumulative.append(cumulative[-1] + hypot(b.x - a.x, b.y - a.y))
    return cumulative


def _nearest_progress(
    points: list[Waypoint], cumulative: list[float], pose: Pose2D
) -> tuple[float, float, float]:
    best_distance_sq = float("inf")
    best_progress = 0.0
    best_lateral_error = 0.0
    for index, (a, b) in enumerate(zip(points[:-1], points[1:])):
        dx, dy = b.x - a.x, b.y - a.y
        length_sq = dx * dx + dy * dy
        if length_sq < 1e-10:
            continue
        fraction = clamp(((pose.x - a.x) * dx + (pose.y - a.y) * dy) / length_sq, 0.0, 1.0)
        px, py = a.x + fraction * dx, a.y + fraction * dy
        distance_sq = (pose.x - px) ** 2 + (pose.y - py) ** 2
        if distance_sq < best_distance_sq:
            best_distance_sq = distance_sq
            best_progress = cumulative[index] + fraction * sqrt(length_sq)
            # Positive error means the vehicle is left of the path direction.
            best_lateral_error = (dx * (pose.y - py) - dy * (pose.x - px)) / sqrt(length_sq)
    return best_progress, best_lateral_error, sqrt(best_distance_sq)


def _sample(points: list[Waypoint], cumulative: list[float], progress: float) -> Waypoint:
    if progress >= cumulative[-1]:
        return points[-1]
    for index in range(len(points) - 1):
        start, end = cumulative[index], cumulative[index + 1]
        if progress > end or end - start < 1e-10:
            continue
        fraction = clamp((progress - start) / (end - start), 0.0, 1.0)
        a, b = points[index], points[index + 1]
        return Waypoint(
            a.x + fraction * (b.x - a.x),
            a.y + fraction * (b.y - a.y),
            wrap_angle(a.yaw + fraction * wrap_angle(b.yaw - a.yaw)),
            a.speed + fraction * (b.speed - a.speed),
        )
    return points[-1]


def calculate_command(
    points: list[Waypoint],
    pose: Pose2D,
    previous_linear: float,
    previous_angular: float,
    dt: float,
    limits: Limits,
) -> Result:
    """Return a bounded forward/turn command; reverse driving is unsupported."""
    if len(points) < 2 or not all(
        isfinite(value)
        for point in points
        for value in (point.x, point.y, point.yaw, point.speed)
    ):
        return Result(0.0, 0.0, "invalid_trajectory")
    if not all(isfinite(value) for value in (pose.x, pose.y, pose.yaw, dt)) or dt <= 0.0:
        return Result(0.0, 0.0, "invalid_pose_or_time")

    cumulative = _cumulative_distances(points)
    if cumulative[-1] < 1e-3:
        return Result(0.0, 0.0, "invalid_trajectory")
    progress, lateral_error, path_distance = _nearest_progress(points, cumulative, pose)
    if path_distance > limits.max_path_distance:
        return Result(0.0, 0.0, "too_far_from_path", lateral_error)
    remaining = cumulative[-1] - progress
    final = points[-1]
    final_stop = abs(final.speed) <= limits.stop_speed_threshold
    goal_distance = hypot(final.x - pose.x, final.y - pose.y)
    goal_heading_error = wrap_angle(final.yaw - pose.yaw)

    # The final point can be spatially near an earlier part of a loop. Require
    # progress near the end before accepting the terminal pose.
    if final_stop and remaining <= max(1.0, limits.lookahead_distance) and (
        goal_distance <= limits.goal_position_tolerance
    ):
        if abs(goal_heading_error) <= limits.goal_yaw_tolerance:
            return Result(0.0, 0.0, "goal_reached", lateral_error, goal_heading_error)
        target_angular = clamp(
            limits.heading_gain * goal_heading_error,
            -limits.max_angular_speed,
            limits.max_angular_speed,
        )
        angular = _rate_limit(target_angular, previous_angular, limits.max_angular_accel * dt)
        return Result(0.0, angular, "align_goal", lateral_error, goal_heading_error)

    lookahead = _sample(
        points, cumulative, min(progress + limits.lookahead_distance, cumulative[-1])
    )
    dx, dy = lookahead.x - pose.x, lookahead.y - pose.y
    local_x = cos(pose.yaw) * dx + sin(pose.yaw) * dy
    local_y = -sin(pose.yaw) * dx + cos(pose.yaw) * dy
    heading_error = atan2(local_y, local_x)

    # A zero-speed waypoint is a stop, not a request to spin toward later path
    # points. Only the terminal goal branch above may rotate while stopped.
    next_stop_distance = None
    for index, point in enumerate(points[1:], start=1):
        if (
            cumulative[index] >= progress - 0.03
            and point.speed <= limits.stop_speed_threshold
        ):
            next_stop_distance = max(cumulative[index] - progress, 0.0)
            break
    if next_stop_distance is not None and next_stop_distance <= 0.05:
        return Result(0.0, 0.0, "planned_stop", lateral_error, heading_error)

    # Sample speed ahead of the vehicle to leave a zero-speed start point,
    # but never sample beyond an upcoming planned stop.
    speed_horizon = limits.lookahead_distance
    if next_stop_distance is not None:
        speed_horizon = min(speed_horizon, max(next_stop_distance - 0.10, 0.0))
    reference_near = _sample(points, cumulative, min(progress + min(0.15, speed_horizon), cumulative[-1]))
    reference_far = _sample(points, cumulative, min(progress + speed_horizon, cumulative[-1]))
    if min(reference_near.speed, reference_far.speed) < -limits.stop_speed_threshold:
        return Result(0.0, 0.0, "reverse_unsupported", lateral_error)
    reference_speed = max(reference_near.speed, reference_far.speed)
    if reference_speed <= limits.stop_speed_threshold:
        return Result(0.0, 0.0, "planned_stop", lateral_error, heading_error)

    if abs(heading_error) > limits.rotate_to_path_threshold:
        target_angular = clamp(
            limits.heading_gain * heading_error,
            -limits.max_angular_speed,
            limits.max_angular_speed,
        )
        angular = _rate_limit(target_angular, previous_angular, limits.max_angular_accel * dt)
        return Result(0.0, angular, "align_path", lateral_error, heading_error)

    distance_sq = local_x * local_x + local_y * local_y
    curvature = 2.0 * local_y / distance_sq if distance_sq > 1e-8 else 0.0
    target_linear = clamp(reference_speed, 0.0, limits.max_linear_speed)
    if next_stop_distance is not None:
        target_linear = min(
            target_linear,
            sqrt(2.0 * limits.max_linear_decel * max(next_stop_distance - 0.03, 0.0)),
        )
    if abs(curvature) > 1e-6:
        target_linear = min(target_linear, limits.max_angular_speed / abs(curvature))

    linear_delta = (
        limits.max_linear_accel if target_linear >= previous_linear else limits.max_linear_decel
    ) * dt
    linear = clamp(_rate_limit(target_linear, previous_linear, linear_delta), 0.0, limits.max_linear_speed)
    target_angular = clamp(
        linear * curvature, -limits.max_angular_speed, limits.max_angular_speed
    )
    angular = _rate_limit(target_angular, previous_angular, limits.max_angular_accel * dt)
    return Result(linear, angular, "track", lateral_error, heading_error)

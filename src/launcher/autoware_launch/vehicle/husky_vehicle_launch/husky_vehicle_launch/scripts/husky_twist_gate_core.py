"""Fail-closed state machine for the Husky Twist vehicle interface."""

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class Velocity:
    linear: float = 0.0
    angular: float = 0.0


class TwistGate:
    def __init__(
        self,
        *,
        allow_output: bool,
        command_timeout: float,
        estop_timeout: float,
        odometry_timeout: float,
        tracking_timeout: float,
        manual_hold: float,
        max_linear: float,
        max_angular: float,
    ):
        if min(command_timeout, estop_timeout, odometry_timeout, tracking_timeout, manual_hold, max_linear, max_angular) <= 0.0:
            raise ValueError("Timeouts and velocity limits must be positive")
        self.allow_output = allow_output
        self.command_timeout = command_timeout
        self.estop_timeout = estop_timeout
        self.odometry_timeout = odometry_timeout
        self.tracking_timeout = tracking_timeout
        self.manual_hold = manual_hold
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.enabled = False
        self.ever_enabled = False
        self.reason = "disabled"
        self.command = Velocity()
        self.command_at = None
        self.estop = None
        self.estop_at = None
        self.odometry_at = None
        self.tracking_at = None
        self.tracking_state = None
        self.manual_at = None
        self.manual_source = None

    @staticmethod
    def _fresh(stamp, now, timeout):
        return stamp is not None and 0.0 <= now - stamp <= timeout

    def set_command(self, linear: float, angular: float, now: float):
        if not all(isfinite(value) for value in (linear, angular)):
            self.command_at = None
            self.disable("invalid_command")
            return
        if linear < 0.0 or abs(linear) > self.max_linear or abs(angular) > self.max_angular:
            self.command_at = None
            self.disable("command_out_of_range")
            return
        self.command = Velocity(linear, angular)
        self.command_at = now

    def set_estop(self, active: bool, now: float):
        self.estop = active
        self.estop_at = now
        if active:
            self.disable("emergency_stop")

    def set_odometry(self, now: float):
        self.odometry_at = now

    def set_tracking(self, state: str, now: float):
        self.tracking_state = state
        self.tracking_at = now
        if self.enabled and state not in ("track", "align_path", "align_goal", "planned_stop", "goal_reached"):
            self.disable("tracking_fault")

    def manual_takeover(self, now: float, source: str = "local"):
        if source not in ("local", "remote"):
            raise ValueError("Manual source must be local or remote")
        self.manual_at = now
        self.manual_source = source
        self.disable("manual_takeover")

    def request_manual(self):
        self.manual_source = "local"
        return self.disable("manual_request")

    def disable(self, reason="disabled"):
        was_enabled = self.enabled
        self.enabled = False
        self.reason = reason
        return was_enabled

    def _block_reason(self, now: float):
        if not self.allow_output:
            return "platform_output_disabled"
        if self.estop is None or not self._fresh(self.estop_at, now, self.estop_timeout):
            return "estop_status_missing_or_stale"
        if self.estop:
            return "emergency_stop"
        if self._fresh(self.manual_at, now, self.manual_hold):
            return "manual_takeover"
        if not self._fresh(self.odometry_at, now, self.odometry_timeout):
            return "odometry_missing_or_stale"
        if not self._fresh(self.tracking_at, now, self.tracking_timeout):
            return "tracking_missing_or_stale"
        if self.tracking_state not in ("track", "align_path", "align_goal", "planned_stop", "goal_reached"):
            return "tracking_fault"
        if not self._fresh(self.command_at, now, self.command_timeout):
            return "command_missing_or_stale"
        return None

    def enable(self, now: float):
        reason = self._block_reason(now)
        if reason:
            self.disable(reason)
            return False, reason
        self.enabled = True
        self.ever_enabled = True
        self.reason = "enabled"
        return True, "enabled"

    def output(self, now: float):
        if not self.enabled:
            return None
        reason = self._block_reason(now)
        if reason:
            self.disable(reason)
            return None
        return self.command

    def mode(self, now: float):
        """Semantic control mode; never infer autonomy from vehicle motion alone."""
        if self.enabled:
            return "autonomous"
        if self.allow_output and (
            self.estop is None or self.estop
            or not self._fresh(self.estop_at, now, self.estop_timeout)
        ):
            return "not_ready"
        if self.reason in ("manual_takeover", "manual_request"):
            return "manual"
        if not self.allow_output:
            return "no_command"
        if not self._fresh(self.odometry_at, now, self.odometry_timeout):
            return "not_ready"
        if self.reason in (
            "command_missing_or_stale", "invalid_command",
            "command_out_of_range", "unsupported_twist_components",
            "simulated_time", "output_topic_conflict", "tracking_fault",
            "tracking_missing_or_stale",
        ):
            return "not_ready"
        return "disengaged"

    def operation_mode(self, now: float):
        """Report the bridge's actual authority and the observed manual source."""
        if self.enabled:
            return "autonomous"
        if self.estop or (
            self.allow_output
            and not self._fresh(self.estop_at, now, self.estop_timeout)
        ):
            return "stop"
        if self.reason in ("manual_takeover", "manual_request"):
            return self.manual_source or "local"
        return "stop"

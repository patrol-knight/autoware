#!/usr/bin/env python3
"""Report localization initialization transitions in the launch terminal."""

import rclpy
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class LocalizationStateLogger(Node):
    def __init__(self):
        super().__init__("localization_state_logger")
        self.previous_state = None
        self.initialization_threshold = None
        self.awaiting_ndt_score = False
        self.initialization_score_reliable = None
        self.ndt_parameters = self.create_client(
            GetParameters, "/localization/pose_estimator/ndt_scan_matcher/get_parameters"
        )
        self.parameter_request_pending = False
        self.create_timer(1.0, self.request_ndt_parameters)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            LocalizationInitializationState,
            "/localization/initialization_state",
            self.on_state,
            qos,
        )
        self.create_subscription(DiagnosticArray, "/diagnostics", self.on_diagnostics, 10)

    def request_ndt_parameters(self):
        if self.initialization_threshold is not None or self.parameter_request_pending:
            return
        if not self.ndt_parameters.service_is_ready():
            return
        request = GetParameters.Request()
        request.names = [
            "score_estimation.converged_param_type",
            "score_estimation.converged_param_transform_probability",
            "score_estimation.converged_param_nearest_voxel_transformation_likelihood",
        ]
        self.parameter_request_pending = True
        self.ndt_parameters.call_async(request).add_done_callback(self.on_ndt_parameters)

    def on_ndt_parameters(self, future):
        self.parameter_request_pending = False
        try:
            values = future.result().values
            if (
                len(values) != 3
                or values[0].type != ParameterType.PARAMETER_INTEGER
                or values[1].type != ParameterType.PARAMETER_DOUBLE
                or values[2].type != ParameterType.PARAMETER_DOUBLE
            ):
                raise ValueError("NDT score parameters are unavailable")
            score_type = values[0].integer_value
            if score_type == 1:
                convergence_threshold = values[2].double_value
                score_name = "nearest voxel transformation likelihood"
            elif score_type == 0:
                convergence_threshold = values[1].double_value
                score_name = "transform probability"
            else:
                raise ValueError(f"Unsupported NDT score type: {score_type}")
            self.initialization_threshold = values[2].double_value
            self.get_logger().info(
                f"NDT initialization NVTL threshold: {self.initialization_threshold:.3f}; "
                f"runtime convergence metric: {score_name}, threshold: {convergence_threshold:.3f}"
            )
        except (AttributeError, RuntimeError, ValueError) as error:
            self.get_logger().warn(f"Could not read NDT score threshold: {error}")

    def on_diagnostics(self, msg):
        if not self.awaiting_ndt_score:
            return
        for status in msg.status:
            if not status.name.endswith("ndt_align_service_status"):
                continue
            values = {item.key: item.value for item in status.values}
            if "is_succeed_service" not in values:
                continue
            self.awaiting_ndt_score = False
            if values["is_succeed_service"] != "True":
                self.get_logger().error(
                    f"NDT initialization alignment failed: {status.message}"
                )
                return
            try:
                score = float(values["best_particle_score"])
            except (KeyError, ValueError):
                self.get_logger().warn("NDT initialization completed without a final score")
                return
            if self.initialization_threshold is None:
                self.get_logger().info(f"Final initialization NDT score: {score:.3f}")
            else:
                reliable = score > self.initialization_threshold
                self.initialization_score_reliable = reliable
                log = self.get_logger().info if reliable else self.get_logger().warn
                log(
                    f"Final initialization NDT NVTL score: {score:.3f}; "
                    f"threshold: {self.initialization_threshold:.3f}; "
                    f"{'reliable' if reliable else 'below threshold / unreliable'}"
                )
            return

    def on_state(self, msg):
        if msg.state == self.previous_state:
            return
        was_initializing = self.previous_state == LocalizationInitializationState.INITIALIZING
        self.previous_state = msg.state
        if msg.state == LocalizationInitializationState.INITIALIZING:
            self.awaiting_ndt_score = True
            self.initialization_score_reliable = None
            self.get_logger().info("Localization initialization in progress")
        elif msg.state == LocalizationInitializationState.INITIALIZED:
            if self.initialization_score_reliable is False:
                self.get_logger().warn(
                    "Initialization completed, but the final NDT score is below its threshold"
                )
            else:
                self.get_logger().info(
                    "Localization initialization completed; verify pointcloud overlap"
                )
        elif msg.state == LocalizationInitializationState.UNINITIALIZED:
            message = "Localization initialization failed" if was_initializing else "Localization is not initialized"
            self.get_logger().warn(message)
        else:
            self.get_logger().warn(f"Localization initialization state: {msg.state}")


def main():
    rclpy.init()
    node = LocalizationStateLogger()
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

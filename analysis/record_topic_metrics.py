#!/usr/bin/env python3
"""Record observed ROS topic frequency and stamped-message age to CSV."""

import argparse
import csv
from collections import deque
from datetime import datetime, timezone
from math import isfinite
from pathlib import Path
import time

import rclpy
from autoware_planning_msgs.msg import Trajectory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


TOPICS = (
    ("/sensing/lidar/concatenated/pointcloud", PointCloud2),
    ("/localization/pose_estimator/pose_with_covariance", PoseWithCovarianceStamped),
    ("/localization/kinematic_state", Odometry),
    ("/planning/trajectory", Trajectory),
    ("/control/husky/cmd_vel", Twist),
    ("/husky/cmd_vel", Twist),
    ("/sensing/lidar/obstacle_stop", Bool),
)
FIELDS = (
    "received_utc", "elapsed_s", "topic", "message_type", "sequence",
    "frequency_hz", "latency_ms", "source_stamp_ns",
)


class TopicMetricsRecorder(Node):
    def __init__(self, writer, output_file, window_size, platform_cmd_topic):
        super().__init__("topic_metrics_recorder")
        self.writer = writer
        self.output_file = output_file
        self.started_at = time.monotonic()
        self.arrivals = {}
        self.counts = {}
        # Best effort subscribers can receive both reliable and best effort publishers.
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        for configured_topic, message_type in TOPICS:
            topic = platform_cmd_topic if configured_topic == "/husky/cmd_vel" else configured_topic
            self.arrivals[topic] = deque(maxlen=window_size)
            self.counts[topic] = 0
            self.create_subscription(
                message_type, topic,
                lambda msg, topic=topic, message_type=message_type:
                    self.record(topic, message_type, msg),
                qos,
            )

    def record(self, topic, message_type, msg):
        received_at = time.monotonic()
        samples = self.arrivals[topic]
        samples.append(received_at)
        frequency_hz = ""
        if len(samples) > 1 and samples[-1] > samples[0]:
            frequency_hz = (len(samples) - 1) / (samples[-1] - samples[0])

        source_stamp_ns = ""
        latency_ms = ""
        header = getattr(msg, "header", None)
        if header is not None:
            stamp = header.stamp
            source_stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            now_ns = self.get_clock().now().nanoseconds
            if source_stamp_ns > 0 and now_ns > 0:
                latency_ms = (now_ns - source_stamp_ns) / 1_000_000

        self.counts[topic] += 1
        self.writer.writerow({
            "received_utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "elapsed_s": f"{received_at - self.started_at:.6f}",
            "topic": topic,
            "message_type": f"{message_type.__module__.split('.')[0]}/msg/{message_type.__name__}",
            "sequence": self.counts[topic],
            "frequency_hz": f"{frequency_hz:.3f}" if frequency_hz != "" else "",
            "latency_ms": f"{latency_ms:.3f}" if latency_ms != "" else "",
            "source_stamp_ns": source_stamp_ns,
        })
        self.output_file.flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("topic_metrics.csv"))
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Seconds to record; 0 means until Ctrl+C")
    parser.add_argument("--window-size", type=int, default=20,
                        help="Number of received messages in the rolling frequency estimate")
    parser.add_argument("--platform-cmd-topic", default="/husky/cmd_vel")
    args, ros_args = parser.parse_known_args()
    if not isfinite(args.duration) or args.duration < 0 or args.window_size < 2:
        parser.error("duration must be nonnegative and window-size must be at least 2")
    if not args.platform_cmd_topic.startswith("/"):
        parser.error("platform-cmd-topic must be an absolute ROS topic")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=ros_args)
    try:
        with args.output.open("w", newline="", encoding="utf-8") as output_file:
            writer = csv.DictWriter(output_file, fieldnames=FIELDS)
            writer.writeheader()
            recorder = TopicMetricsRecorder(
                writer, output_file, args.window_size, args.platform_cmd_topic
            )
            try:
                print(f"Recording {len(TOPICS)} topics to {args.output}; Ctrl+C to stop.")
                deadline = time.monotonic() + args.duration if args.duration else None
                while rclpy.ok() and (deadline is None or time.monotonic() < deadline):
                    rclpy.spin_once(recorder, timeout_sec=0.2)
            except KeyboardInterrupt:
                pass
            finally:
                recorder.destroy_node()
        print(f"Saved {args.output}")
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Export one /planning/trajectory message from a ROS 2 SQLite bag for XY plotting."""

import argparse
import csv
import sqlite3
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="ROS 2 bag directory or its .db3 file")
    parser.add_argument(
        "--time", type=float, required=True,
        help="Select the trajectory closest to this many seconds after bag start",
    )
    parser.add_argument("--output", type=Path, help="CSV output path")
    args = parser.parse_args()

    db = args.bag
    if db.is_dir():
        candidates = sorted(db.glob("*.db3"))
        if len(candidates) != 1:
            parser.error("bag directory must contain exactly one .db3 file")
        db = candidates[0]
    if not db.is_file():
        parser.error(f"bag database does not exist: {db}")

    conn = sqlite3.connect(f"file:{db.resolve()}?mode=ro", uri=True)
    try:
        start_ns = conn.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
        row = conn.execute(
            "SELECT id, type FROM topics WHERE name = '/planning/trajectory'"
        ).fetchone()
        if row is None:
            parser.error("/planning/trajectory is absent from this bag")
        topic_id, type_name = row
        target_ns = start_ns + round(args.time * 1e9)
        selected = conn.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ? "
            "ORDER BY ABS(timestamp - ?) LIMIT 1",
            (topic_id, target_ns),
        ).fetchone()
        if selected is None:
            parser.error("/planning/trajectory has no messages in this bag")
        stamp_ns, blob = selected
        trajectory = deserialize_message(blob, get_message(type_name))
    finally:
        conn.close()

    if not trajectory.points:
        parser.error("the selected trajectory has no points")
    output = args.output or Path(f"analysis/planning_trajectory_{args.time:g}s.csv")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("point_index", "x_m", "y_m", "z_m", "speed_mps"))
        for index, point in enumerate(trajectory.points):
            position = point.pose.position
            writer.writerow((
                index, position.x, position.y, position.z,
                point.longitudinal_velocity_mps,
            ))
    print(
        f"Saved {len(trajectory.points)} points in {trajectory.header.frame_id} "
        f"at bag +{(stamp_ns - start_ns) / 1e9:.3f} s to {output}"
    )


if __name__ == "__main__":
    main()

import sys
import unittest
from pathlib import Path

import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from lidar_obstacle_stop import LidarObstacleStop


class TestLidarObstacleNode(unittest.TestCase):
    def test_timer_processes_only_latest_unprocessed_cloud(self):
        rclpy.init(args=["--ros-args", "-p", "detection_fps:=5.0"])
        node = LidarObstacleStop()
        try:
            self.assertEqual(node.detection_fps, 5.0)
            self.assertEqual(next(node.timers).timer_period_ns, 200_000_000)
            processed = []
            node.process_cloud = processed.append
            first, second, third = object(), object(), object()
            node.on_cloud(first)
            node.on_cloud(second)
            node.on_detection_timer()
            node.on_detection_timer()
            node.on_cloud(third)
            node.on_detection_timer()
            self.assertEqual(processed, [second, third])
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()

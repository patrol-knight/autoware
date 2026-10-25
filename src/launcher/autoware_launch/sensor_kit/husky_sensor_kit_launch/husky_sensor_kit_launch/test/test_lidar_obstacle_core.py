import sys
import unittest
from math import cos, sin
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from lidar_obstacle_core import count_front_obstacles, count_obstacles, segment_ground


class TestLidarObstacleCore(unittest.TestCase):
    def test_adjacent_low_rings_remove_flat_ground(self):
        points = [(1.0, 0.0, -1.0, 0), (1.2, 0.0, -1.0, 1)]
        self.assertEqual(count_obstacles(points), 0)

    def test_steep_pair_and_missing_neighbor_remain_obstacles(self):
        points = [(1.0, 0.0, -1.0, 0), (1.1, 0.0, -0.5, 1)]
        self.assertEqual(count_obstacles(points), 2)
        self.assertEqual(count_obstacles([(1.0, 0.0, -0.5, 1)]), 1)

    def test_front_range_and_twenty_point_stop_boundary(self):
        points = [(cos(i * 0.01), sin(i * 0.01), -0.5, 9)
                  for i in range(19)]
        points += [(-1.0, 0.0, -0.5, 9), (2.1, 0.0, -0.5, 9)]
        self.assertEqual(count_obstacles(points), 19)
        points.append((cos(0.19), sin(0.19), -0.5, 9))
        self.assertEqual(count_obstacles(points), 20)

    def test_first_valid_return_is_used_per_cell(self):
        points = [(1.5, 0.0, 0.1, 9), (1.0, 0.0, 0.2, 9)]
        _ground, nonground = segment_ground(points)
        self.assertEqual(nonground[0, 0], np.float32(1.5))
        self.assertEqual(count_obstacles(points), 1)
        self.assertEqual(count_obstacles([(float("nan"), 0.0, 0.0, 9)]), 0)

    def test_later_return_is_ignored_even_if_closer(self):
        dtype = [("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                 ("channel", "u1")]
        points = np.array([(3.0, 0.0, 0.1, 9), (1.0, 0.0, 0.2, 9)], dtype=dtype)
        _ground, nonground = segment_ground(points)
        self.assertEqual(nonground[0, 2], np.float32(0.1))
        self.assertEqual(count_front_obstacles(nonground), 0)

    def test_ground_and_nonground_outputs_cover_the_projected_scan(self):
        points = [
            (1.0, 0.0, -1.0, 0), (1.2, 0.0, -1.0, 1),
            (1.0, 0.2, -0.5, 9), (-1.0, 0.0, -0.5, 9),
        ]
        ground, nonground = segment_ground(points)
        self.assertEqual(len(ground), 2)
        self.assertEqual(len(nonground), 2)
        self.assertEqual(count_front_obstacles(nonground), 1)

    def test_structured_driver_cloud_and_ring_alias(self):
        cloud = np.array(
            [(1.0, 0.0, -1.0, 0), (1.2, 0.0, -1.0, 1),
             (1.0, 0.2, -0.5, 9)],
            dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                   ("ring", "<u2")],
        )
        ground, nonground = segment_ground(cloud)
        self.assertEqual(ground.shape, (2, 3))
        self.assertEqual(nonground.shape, (1, 3))
        self.assertEqual(ground.dtype, np.float32)
        self.assertEqual(count_front_obstacles(nonground), 1)


if __name__ == "__main__":
    unittest.main()

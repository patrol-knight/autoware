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

    def test_ground_and_nonground_outputs_cover_only_the_front_sector(self):
        points = [
            (1.0, 0.0, -1.0, 0), (1.2, 0.0, -1.0, 1),
            (1.0, 0.2, -0.5, 9), (-1.0, 0.0, -0.5, 9),
        ]
        ground, nonground = segment_ground(points)
        self.assertEqual(len(ground), 2)
        self.assertEqual(len(nonground), 1)
        self.assertEqual(count_front_obstacles(nonground), 1)

    def test_ground_segmentation_keeps_five_degree_margin(self):
        def pair(degrees):
            angle = np.radians(degrees)
            return [(cos(angle), sin(angle), -1.0, 0),
                    (1.2 * cos(angle), 1.2 * sin(angle), -1.0, 1)]

        angle = np.radians(49)
        points = pair(30) + pair(-30) + pair(49) + pair(51)
        points.append((cos(angle), sin(angle), -0.5, 9))
        ground, nonground = segment_ground(points, half_angle_deg=45.0)
        self.assertEqual(len(ground), 6)
        self.assertEqual(len(nonground), 1)
        self.assertTrue(np.all(ground[:, 0] > 0))
        self.assertEqual(count_front_obstacles(nonground, half_angle_deg=45.0), 0)

        outside_ground, outside_nonground = segment_ground(pair(51), half_angle_deg=45.0)
        self.assertEqual(outside_ground.shape, (0, 3))
        self.assertEqual(outside_nonground.shape, (0, 3))

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

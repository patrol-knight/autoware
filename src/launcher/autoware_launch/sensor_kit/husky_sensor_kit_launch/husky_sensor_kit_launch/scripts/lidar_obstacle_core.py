"""Vectorized ring based ground removal and front obstacle counting."""

import numpy as np


def _point_columns(points):
    """Accept a PointCloud2 structured array or iterable of x/y/z/channel rows."""
    if isinstance(points, np.ndarray) and points.dtype.names:
        channel_name = "channel" if "channel" in points.dtype.names else "ring"
        return tuple(points[name] for name in ("x", "y", "z", channel_name))
    array = np.asarray(list(points), dtype=np.float64)
    if array.size == 0:
        array = array.reshape(0, 4)
    if array.ndim != 2 or array.shape[1] != 4:
        raise ValueError("Points must have x, y, z, and channel columns")
    return tuple(array[:, column] for column in range(4))


def segment_ground(
    points, *, num_scans=16, horizon_scan=1800, ground_scan_index=7,
    sensor_mount_angle_deg=0.0, ground_angle_tolerance_deg=10.0,
    half_angle_deg=60.0,
):
    """Return ground and non-ground XYZ cells within the forward sector plus 5°.

    Channel 0 is the lowest elevation. Each ring/azimuth cell keeps its
    first valid return; adjacent low rings are classified by their local slope.
    """
    if not (num_scans > 1 and horizon_scan > 0 and
            0 <= ground_scan_index < num_scans - 1 and
            ground_angle_tolerance_deg >= 0 and 0 < half_angle_deg <= 90):
        raise ValueError("Invalid ground removal parameters")

    x, y, z, channel = _point_columns(points)
    valid = (np.isfinite(x) & np.isfinite(y) & np.isfinite(z) &
             np.isfinite(channel) & (channel >= 0) & (channel < num_scans))
    x, y, z, channel = x[valid], y[valid], z[valid], channel[valid]
    ring = channel.astype(np.int32)
    valid_ring = ring == channel
    x, y, z, ring = x[valid_ring], y[valid_ring], z[valid_ring], ring[valid_ring]
    range_sq = x * x + y * y
    valid_range = range_sq > 0
    x, y, z, ring = x[valid_range], y[valid_range], z[valid_range], ring[valid_range]
    range_sq = range_sq[valid_range]
    if not len(x):
        empty = np.empty((0, 3), dtype=np.float32)
        return empty, empty.copy()

    azimuth = np.arctan2(y.astype(np.float64), x.astype(np.float64))
    in_sector = np.abs(np.degrees(azimuth)) <= half_angle_deg + 5.0
    x, y, z, ring = x[in_sector], y[in_sector], z[in_sector], ring[in_sector]
    azimuth = azimuth[in_sector]
    if not len(x):
        empty = np.empty((0, 3), dtype=np.float32)
        return empty, empty.copy()
    column = (np.floor((azimuth + np.pi) / (2 * np.pi / horizon_scan))
              .astype(np.int32) % horizon_scan)
    key = ring * horizon_scan + column
    cell_count = num_scans * horizon_scan
    # Keep the earliest input index in each occupied cell; later returns are ignored.
    selected_index = np.full(cell_count, len(key), dtype=np.intp)
    np.minimum.at(selected_index, key, np.arange(len(key)))
    keys = np.flatnonzero(selected_index != len(key))
    selected = selected_index[keys]
    xyz = np.column_stack((x[selected], y[selected], z[selected])).astype(np.float32)

    # Compare adjacent rows of the organized scan only where both cells exist.
    grid = np.empty((cell_count, 3), dtype=np.float32)
    grid[keys] = xyz
    present = np.zeros(cell_count, dtype=bool)
    present[keys] = True
    lower = keys[keys < (ground_scan_index + 1) * horizon_scan]
    lower = lower[present[lower] & present[lower + horizon_scan]]
    upper = lower + horizon_scan
    delta = grid[upper] - grid[lower]
    delta = delta.astype(np.float64)
    angle = np.degrees(np.arctan2(delta[:, 2], np.hypot(delta[:, 0], delta[:, 1])))
    matched = np.abs(angle - sensor_mount_angle_deg) <= ground_angle_tolerance_deg
    ground_cells = np.zeros(cell_count, dtype=bool)
    ground_cells[lower[matched]] = True
    ground_cells[upper[matched]] = True
    is_ground = ground_cells[keys]
    return np.ascontiguousarray(xyz[is_ground]), np.ascontiguousarray(xyz[~is_ground])


def count_front_obstacles(points, *, max_range=2.0, half_angle_deg=60.0,
                          threshold=20):
    """Count front non-ground cells, capped at the stop threshold."""
    if max_range <= 0 or not 0 < half_angle_deg <= 90 or threshold <= 0:
        raise ValueError("Invalid obstacle counting parameters")
    if not len(points):
        return 0
    xyz = np.asarray(points)
    x, y = xyz[:, 0], xyz[:, 1]
    in_range = x * x + y * y <= max_range * max_range
    in_front = np.abs(np.degrees(np.arctan2(
        y.astype(np.float64), x.astype(np.float64)))) <= half_angle_deg
    return min(int(np.count_nonzero(in_range & in_front)), threshold)


def count_obstacles(points, *, num_scans=16, horizon_scan=1800,
                    ground_scan_index=7, sensor_mount_angle_deg=0.0,
                    ground_angle_tolerance_deg=10.0, max_range=2.0,
                    half_angle_deg=60.0, threshold=20):
    """Convenience wrapper for segmentation followed by obstacle counting."""
    _ground, nonground = segment_ground(
        points, num_scans=num_scans, horizon_scan=horizon_scan,
        ground_scan_index=ground_scan_index,
        sensor_mount_angle_deg=sensor_mount_angle_deg,
        ground_angle_tolerance_deg=ground_angle_tolerance_deg,
        half_angle_deg=half_angle_deg,
    )
    return count_front_obstacles(nonground, max_range=max_range,
                                 half_angle_deg=half_angle_deg,
                                 threshold=threshold)

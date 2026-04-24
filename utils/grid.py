"""
utils/grid.py — Occupancy-grid utilities.

Converts ROS OccupancyGrid data into NumPy matrices, marks obstacles,
and inflates them by the robot radius so the planner treats the robot
as a point.
"""

import numpy as np
from typing import Tuple


def occupancy_to_matrix(
    data: list,
    width: int,
    height: int,
) -> np.ndarray:
    """
    Reshape the flat ``OccupancyGrid.data`` list into a 2-D cost matrix.

    Parameters
    ----------
    data : list[int]
        Flat occupancy values (−1 = unknown, 0–100 = cost).
    width, height : grid dimensions published by SLAM.

    Returns
    -------
    np.ndarray of shape (height, width) with dtype int8.
    """
    grid = np.array(data, dtype=np.int8).reshape((height, width))
    return grid


def mark_obstacles(
    grid: np.ndarray,
    threshold: int = 65,
    unknown_as_blocked: bool = True,
) -> np.ndarray:
    """
    Return a boolean *blocked* matrix.

    A cell is blocked if:
    - its cost ≥ ``threshold``, **or**
    - its cost == −1 (unknown) and ``unknown_as_blocked`` is True.

    Parameters
    ----------
    grid : 2-D int8 cost matrix from :func:`occupancy_to_matrix`.
    threshold : cost value at or above which a cell is considered occupied.
    unknown_as_blocked : whether to treat unknown (−1) cells as blocked.

    Returns
    -------
    np.ndarray of bool, same shape as *grid*.
    """
    blocked = grid >= threshold
    if unknown_as_blocked:
        blocked = blocked | (grid == -1)
    return blocked


def inflate_obstacles(
    blocked: np.ndarray,
    radius_cells: int,
) -> np.ndarray:
    """
    Dilate the blocked mask by *radius_cells* using a circular kernel.

    This effectively grows every obstacle by the robot's radius so that
    the A* planner can treat the robot as a dimensionless point.

    Parameters
    ----------
    blocked : boolean obstacle matrix.
    radius_cells : inflation radius in grid cells.

    Returns
    -------
    np.ndarray of bool — inflated blocked matrix.
    """
    if radius_cells <= 0:
        return blocked.copy()

    from scipy.ndimage import binary_dilation

    # Build a circular structuring element
    diameter = 2 * radius_cells + 1
    y, x = np.ogrid[
        -radius_cells : radius_cells + 1,
        -radius_cells : radius_cells + 1,
    ]
    kernel = (x * x + y * y) <= (radius_cells * radius_cells)

    inflated = binary_dilation(blocked, structure=kernel)
    return inflated.astype(bool)


def grid_bounds_check(
    gx: int,
    gy: int,
    width: int,
    height: int,
) -> bool:
    """Return True if (gx, gy) is inside the grid."""
    return 0 <= gx < width and 0 <= gy < height

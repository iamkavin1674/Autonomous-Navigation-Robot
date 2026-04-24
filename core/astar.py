"""
core/astar.py — A* global planner on an 8-connected occupancy grid.

Usage
-----
    from core.astar import astar

    path = astar(blocked_grid, (sx, sy), (gx, gy))
    # path is a list of (col, row) tuples, or None if unreachable.
"""

import heapq
import math
import time
from typing import List, Optional, Tuple

import numpy as np

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import PLANNING_TIMEOUT


# ──────────────────────────────────────────────
#  Heuristic
# ──────────────────────────────────────────────

# Precomputed constants for octile distance
_SQRT2 = math.sqrt(2.0)
_DIAG_COST = _SQRT2
_CARD_COST = 1.0


def _heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    """
    Octile distance — admissible heuristic for 8-connected grids.

    h = max(|dx|, |dy|) + (√2 − 1) · min(|dx|, |dy|)
    """
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return max(dx, dy) + (_SQRT2 - 1.0) * min(dx, dy)


# ──────────────────────────────────────────────
#  8-connected neighbour offsets
# ──────────────────────────────────────────────

# (dx, dy, cost)
_NEIGHBOURS = [
    ( 1,  0, _CARD_COST),
    (-1,  0, _CARD_COST),
    ( 0,  1, _CARD_COST),
    ( 0, -1, _CARD_COST),
    ( 1,  1, _DIAG_COST),
    ( 1, -1, _DIAG_COST),
    (-1,  1, _DIAG_COST),
    (-1, -1, _DIAG_COST),
]


# ──────────────────────────────────────────────
#  A* Search
# ──────────────────────────────────────────────

def astar(
    blocked: np.ndarray,
    start: Tuple[int, int],
    goal: Tuple[int, int],
    timeout: float = PLANNING_TIMEOUT,
) -> Optional[List[Tuple[int, int]]]:
    """
    Run A* on an 8-connected grid.

    Parameters
    ----------
    blocked : 2-D boolean array (height × width).
        ``True`` ⇒ impassable cell.
    start : (x, y) — column, row of the starting cell.
    goal : (x, y) — column, row of the goal cell.
    timeout : seconds before the planner gives up.

    Returns
    -------
    list[(x, y)] from start to goal inclusive, or ``None`` if the goal
    is unreachable or the timeout fires.
    """
    height, width = blocked.shape

    # Quick sanity checks
    sx, sy = start
    gx, gy = goal
    if not (0 <= sx < width and 0 <= sy < height):
        return None
    if not (0 <= gx < width and 0 <= gy < height):
        return None
    if blocked[sy, sx]:
        return None
    if blocked[gy, gx]:
        return None
    if start == goal:
        return [start]

    t0 = time.monotonic()

    # Priority queue: (f_cost, tiebreak_counter, (x, y))
    counter = 0
    open_set: list = []
    heapq.heappush(open_set, (0.0 + _heuristic(start, goal), counter, start))
    counter += 1

    came_from: dict = {}
    g_score: dict = {start: 0.0}

    closed: set = set()

    while open_set:
        # Timeout guard
        if time.monotonic() - t0 > timeout:
            return None

        f, _, current = heapq.heappop(open_set)
        if current in closed:
            continue
        closed.add(current)

        if current == goal:
            return _reconstruct(came_from, current)

        cx, cy = current
        current_g = g_score[current]

        for dx, dy, move_cost in _NEIGHBOURS:
            nx, ny = cx + dx, cy + dy

            # Bounds check
            if not (0 <= nx < width and 0 <= ny < height):
                continue
            # Obstacle check
            if blocked[ny, nx]:
                continue
            # Corner-cutting safety: for diagonal moves, both adjacent
            # cardinal cells must be free to avoid clipping corners.
            if dx != 0 and dy != 0:
                if blocked[cy, cx + dx] or blocked[cy + dy, cx]:
                    continue

            neighbour = (nx, ny)
            if neighbour in closed:
                continue

            tentative_g = current_g + move_cost
            if tentative_g < g_score.get(neighbour, float("inf")):
                g_score[neighbour] = tentative_g
                f_score = tentative_g + _heuristic(neighbour, goal)
                came_from[neighbour] = current
                heapq.heappush(open_set, (f_score, counter, neighbour))
                counter += 1

    # Open set exhausted — goal unreachable
    return None


def _reconstruct(
    came_from: dict,
    current: Tuple[int, int],
) -> List[Tuple[int, int]]:
    """Walk back through *came_from* to build the path."""
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

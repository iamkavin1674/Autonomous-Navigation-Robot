"""
config.py — Central configuration for the Autonomous Navigation Robot.

All tuneable parameters are stored here as module-level constants
grouped by subsystem.  Every other module imports from this file:

    from config import (
        MAX_LINEAR_SPEED, LOOKAHEAD_DISTANCE, ...
    )
"""

from dataclasses import dataclass, field
from typing import List
import math


# ─────────────────────────────────────────────
#  Robot Physical Parameters
# ─────────────────────────────────────────────
ROBOT_RADIUS: float = 0.18          # metres — used for obstacle inflation
MAX_LINEAR_SPEED: float = 0.35      # m/s — absolute cap on vx, vy
MAX_ANGULAR_SPEED: float = 1.0      # rad/s — absolute cap on wz

# ─────────────────────────────────────────────
#  TF Frame IDs
# ─────────────────────────────────────────────
MAP_FRAME: str = "map"
ODOM_FRAME: str = "odom"
BASE_FRAME: str = "base_link"

# ─────────────────────────────────────────────
#  Global Planner (A*)
# ─────────────────────────────────────────────
GRID_RESOLUTION: float = 0.05       # metres per cell (matches SLAM default)
OBSTACLE_THRESHOLD: int = 65        # OccupancyGrid value ≥ this → blocked
UNKNOWN_AS_OBSTACLE: bool = True    # treat −1 cells as blocked
PLANNING_TIMEOUT: float = 5.0       # seconds — give up if A* takes longer
INFLATION_RADIUS_CELLS: int = 4     # inflate obstacles by this many cells

# ─────────────────────────────────────────────
#  Path Post-Processing
# ─────────────────────────────────────────────
PATH_MIN_POINT_DISTANCE: float = 0.03          # metres — prune closer points
PATH_SMOOTH_WEIGHT_DATA: float = 0.1           # how much to keep original path
PATH_SMOOTH_WEIGHT_SMOOTH: float = 0.3         # how much to smooth
PATH_SMOOTH_TOLERANCE: float = 0.00001         # convergence threshold
PATH_COLLINEAR_ANGLE_THRESHOLD: float = 0.15   # radians (~8.6°)

# ─────────────────────────────────────────────
#  Local Controller (Regulated Pure Pursuit – Omni)
# ─────────────────────────────────────────────
CONTROL_HZ: float = 20.0                          # control loop frequency
LOOKAHEAD_DISTANCE: float = 0.30                   # metres
GOAL_TOLERANCE: float = 0.08                       # metres — "arrived" radius
HEADING_KP: float = 1.5                            # proportional gain on yaw error
APPROACH_VELOCITY_SCALING: float = 0.5             # slow down inside this radius
REGULATE_MIN_SPEED: float = 0.05                   # never command less than this (unless stopping)
CROSS_TRACK_ERROR_THRESHOLD: float = 0.40          # metres — replan if exceeded

# ─────────────────────────────────────────────
#  Sensor Fusion
# ─────────────────────────────────────────────
NUM_IR_SENSORS: int = 8
NUM_ULTRASONIC_SENSORS: int = 8

IR_WEIGHT: float = 0.35
ULTRASONIC_WEIGHT: float = 0.40
CAMERA_WEIGHT: float = 0.25

DANGER_THRESHOLD: float = 0.70                # danger score ≥ this → emergency stop
OBSTACLE_STOP_DISTANCE: float = 0.12          # metres — full stop
OBSTACLE_SLOW_DISTANCE: float = 0.45          # metres — begin tapering speed

SENSOR_FUSION_HZ: float = 20.0               # publish rate

# IR / ultrasonic Range message defaults
IR_MIN_RANGE: float = 0.02
IR_MAX_RANGE: float = 0.80
ULTRASONIC_MIN_RANGE: float = 0.02
ULTRASONIC_MAX_RANGE: float = 4.0

# ─────────────────────────────────────────────
#  Camera / OpenCV
# ─────────────────────────────────────────────
CAMERA_TOPIC: str = "/esp32_camera/image_raw"
CANNY_LOW: int = 50
CANNY_HIGH: int = 150
CAMERA_OBSTACLE_ROI_TOP_RATIO: float = 0.5    # only look at the bottom half

# ─────────────────────────────────────────────
#  Recovery Behaviour
# ─────────────────────────────────────────────
MAX_RECOVERY_ATTEMPTS: int = 3
RECOVERY_ROTATION_SPEED: float = 0.4          # rad/s
RECOVERY_ROTATION_DURATION: float = 3.0       # seconds — one full sweep
RECOVERY_BACKUP_SPEED: float = -0.10          # m/s
RECOVERY_BACKUP_DURATION: float = 1.0         # seconds
RECOVERY_PAUSE_DURATION: float = 1.0          # seconds — wait after stop

# ─────────────────────────────────────────────
#  Nav2 Action Client
# ─────────────────────────────────────────────
NAV2_ACTION_NAME: str = "navigate_to_pose"
NAV2_ACTION_TIMEOUT: float = 300.0            # seconds

# ─────────────────────────────────────────────
#  SLAM Toolbox
# ─────────────────────────────────────────────
SLAM_MAP_TOPIC: str = "/map"
SLAM_SAVE_MAP_SERVICE: str = "/slam_toolbox/save_map"

# ─────────────────────────────────────────────
#  Replanning
# ─────────────────────────────────────────────
REPLAN_DANGER_SPIKE: float = 0.60             # danger score jump triggers replan
REPLAN_COOLDOWN: float = 2.0                  # seconds between replans

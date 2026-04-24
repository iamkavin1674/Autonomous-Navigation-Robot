"""
core/fusion.py — Sensor fusion logic for IR + ultrasonic + ESP32 camera.

Combines readings from 8 IR rangers, 8 ultrasonic rangers, and a
single camera into a unified danger score and proximity-based speed
multiplier.  No ROS node code here — just data processing.
"""

import math
from typing import List, Optional

import cv2
import numpy as np

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import (
    NUM_IR_SENSORS,
    NUM_ULTRASONIC_SENSORS,
    IR_WEIGHT,
    ULTRASONIC_WEIGHT,
    CAMERA_WEIGHT,
    IR_MIN_RANGE,
    IR_MAX_RANGE,
    ULTRASONIC_MIN_RANGE,
    ULTRASONIC_MAX_RANGE,
    DANGER_THRESHOLD,
    OBSTACLE_STOP_DISTANCE,
    OBSTACLE_SLOW_DISTANCE,
    CANNY_LOW,
    CANNY_HIGH,
    CAMERA_OBSTACLE_ROI_TOP_RATIO,
)


class SensorFusion:
    """
    Fuses IR, ultrasonic, and camera data into obstacle-awareness
    metrics consumed by the navigation controller.
    """

    def __init__(self) -> None:
        # Latest range readings (metres).  Initialised to max range
        # (= "nothing detected") so the robot doesn't freeze on boot.
        self._ir_ranges: List[float] = [IR_MAX_RANGE] * NUM_IR_SENSORS
        self._us_ranges: List[float] = [ULTRASONIC_MAX_RANGE] * NUM_ULTRASONIC_SENSORS

        # Camera-derived obstacle density (0.0 – 1.0)
        self._camera_score: float = 0.0

    # ──────────────────────────────────────────
    #  Update Methods
    # ──────────────────────────────────────────

    def update_ir(self, sensor_index: int, range_m: float) -> None:
        """
        Store a single IR reading.

        Parameters
        ----------
        sensor_index : 0-based index of the IR sensor.
        range_m : measured distance in metres.
        """
        if 0 <= sensor_index < NUM_IR_SENSORS:
            # Clamp to valid range
            clamped = max(IR_MIN_RANGE, min(range_m, IR_MAX_RANGE))
            self._ir_ranges[sensor_index] = clamped

    def update_ultrasonic(self, sensor_index: int, range_m: float) -> None:
        """
        Store a single ultrasonic reading.

        Parameters
        ----------
        sensor_index : 0-based index of the ultrasonic sensor.
        range_m : measured distance in metres.
        """
        if 0 <= sensor_index < NUM_ULTRASONIC_SENSORS:
            clamped = max(ULTRASONIC_MIN_RANGE, min(range_m, ULTRASONIC_MAX_RANGE))
            self._us_ranges[sensor_index] = clamped

    def update_camera(self, frame: np.ndarray) -> None:
        """
        Run simple obstacle detection on a camera frame.

        Strategy:
        1. Convert to grayscale.
        2. Gaussian blur to suppress noise.
        3. Canny edge detection.
        4. Count edge pixels in the *bottom half* of the frame (ROI
           where nearby obstacles are most visible).
        5. Normalise to 0.0–1.0 where 1.0 = very dense edges = likely
           obstacle filling the view.
        """
        if frame is None or frame.size == 0:
            return

        # Grayscale
        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame

        # Blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Canny edges
        edges = cv2.Canny(blurred, CANNY_LOW, CANNY_HIGH)

        # ROI — bottom portion of the image
        h = edges.shape[0]
        roi_top = int(h * CAMERA_OBSTACLE_ROI_TOP_RATIO)
        roi = edges[roi_top:, :]

        # Normalise edge density → 0.0 – 1.0
        total_pixels = roi.size
        if total_pixels == 0:
            self._camera_score = 0.0
            return

        edge_count = np.count_nonzero(roi)
        # Empirical cap: if ≥ 30 % of pixels are edges, saturate at 1.0
        raw = edge_count / total_pixels
        self._camera_score = min(raw / 0.30, 1.0)

    # ──────────────────────────────────────────
    #  Query Methods
    # ──────────────────────────────────────────

    def get_danger_score(self) -> float:
        """
        Weighted combination of all sensor sources.

        Returns a value in [0.0, 1.0]:
        - 0.0 → all clear
        - ≥ DANGER_THRESHOLD → imminent collision / emergency stop
        """
        # IR danger: normalise min reading against range window
        ir_min = min(self._ir_ranges)
        ir_danger = 1.0 - _range_normalise(
            ir_min, OBSTACLE_STOP_DISTANCE, IR_MAX_RANGE
        )

        # Ultrasonic danger
        us_min = min(self._us_ranges)
        us_danger = 1.0 - _range_normalise(
            us_min, OBSTACLE_STOP_DISTANCE, ULTRASONIC_MAX_RANGE
        )

        # Weighted sum
        score = (
            IR_WEIGHT * ir_danger
            + ULTRASONIC_WEIGHT * us_danger
            + CAMERA_WEIGHT * self._camera_score
        )
        return max(0.0, min(1.0, score))

    def get_min_range(self) -> float:
        """Return the smallest distance across all range sensors."""
        return min(min(self._ir_ranges), min(self._us_ranges))

    def get_proximity_factor(self) -> float:
        """
        Speed multiplier based on the nearest obstacle.

        Returns
        -------
        float in [0.0, 1.0]:
            1.0 → clear, full speed allowed.
            0.0 → obstacle at stop distance, must halt.

        The factor linearly tapers between ``OBSTACLE_SLOW_DISTANCE``
        and ``OBSTACLE_STOP_DISTANCE``.
        """
        nearest = self.get_min_range()

        if nearest >= OBSTACLE_SLOW_DISTANCE:
            return 1.0
        if nearest <= OBSTACLE_STOP_DISTANCE:
            return 0.0

        # Linear interpolation
        return (nearest - OBSTACLE_STOP_DISTANCE) / (
            OBSTACLE_SLOW_DISTANCE - OBSTACLE_STOP_DISTANCE
        )

    def is_emergency(self) -> bool:
        """True when the danger score exceeds the threshold."""
        return self.get_danger_score() >= DANGER_THRESHOLD


# ──────────────────────────────────────────────
#  Private Helpers
# ──────────────────────────────────────────────

def _range_normalise(value: float, low: float, high: float) -> float:
    """
    Normalise *value* into [0.0, 1.0] given *low* and *high* bounds.
    Values below *low* → 0.0;  values above *high* → 1.0.
    """
    if high <= low:
        return 0.0
    return max(0.0, min(1.0, (value - low) / (high - low)))

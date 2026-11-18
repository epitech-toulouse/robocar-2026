"""Shared input preprocessing for ROS BC data collection, training, and runtime."""

from __future__ import annotations

import math

import cv2
import numpy as np


IMAGE_WIDTH = 128
IMAGE_HEIGHT = 96
SCAN_BINS = 180


def image_to_model_input(image: np.ndarray) -> np.ndarray:
    """Convert a BGR or grayscale image to normalized [1, H, W] float data."""
    if image is None or image.size == 0:
        raise ValueError("camera image is empty")
    if image.ndim == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    elif image.ndim == 2:
        gray = image
    else:
        raise ValueError(f"unsupported camera image shape: {image.shape}")
    resized = cv2.resize(gray, (IMAGE_WIDTH, IMAGE_HEIGHT), interpolation=cv2.INTER_AREA)
    return resized.astype(np.float32)[None, :, :] / 255.0


def scan_to_model_input(
    ranges,
    angle_min: float,
    angle_increment: float,
    max_range_m: float,
    scan_bins: int = SCAN_BINS,
) -> np.ndarray:
    """Bin the forward 180-degree LiDAR sector into normalized nearest ranges."""
    if max_range_m <= 0 or scan_bins <= 0:
        raise ValueError("max_range_m and scan_bins must be positive")
    if not math.isfinite(angle_min) or not math.isfinite(angle_increment):
        raise ValueError("LiDAR angles must be finite")
    bins = np.full(scan_bins, max_range_m, dtype=np.float32)
    if angle_increment == 0.0:
        return bins / max_range_m

    for index, raw_distance in enumerate(ranges):
        angle = math.atan2(
            math.sin(angle_min + index * angle_increment),
            math.cos(angle_min + index * angle_increment),
        )
        if angle < -math.pi / 2 or angle > math.pi / 2:
            continue
        try:
            distance = float(raw_distance)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(distance) or distance <= 0.0:
            continue
        bin_index = int(round((angle + math.pi / 2) * (scan_bins - 1) / math.pi))
        bins[bin_index] = min(bins[bin_index], min(distance, max_range_m))
    return bins / max_range_m

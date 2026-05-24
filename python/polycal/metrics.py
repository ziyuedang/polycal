"""Evaluation metrics for calibration uncertainty intervals."""

from __future__ import annotations

import numpy as np


def picp(intervals: np.ndarray, ground_truths: np.ndarray) -> np.ndarray:
    """Compute Prediction Interval Coverage Probability per DOF.

    Args:
        intervals: Prediction intervals with shape ``(N, 6, 2)``. The last
            dimension stores lower and upper bounds.
        ground_truths: Ground-truth values with shape ``(N, 6)``.

    Returns:
        Coverage fraction per DOF with shape ``(6,)``.

    Raises:
        ValueError: If input shapes are invalid or interval bounds are ordered
            incorrectly.

    Units:
        DOF order is ``[X_cm, Y_cm, Z_cm, Roll_deg, Pitch_deg, Yaw_deg]``.
    """
    interval_array = _validate_intervals(intervals)
    ground_truth_array = _validate_ground_truths(ground_truths, interval_array.shape[0])
    lower = interval_array[:, :, 0]
    upper = interval_array[:, :, 1]
    covered = (ground_truth_array >= lower) & (ground_truth_array <= upper)
    return covered.mean(axis=0)


def mpiw(intervals: np.ndarray) -> np.ndarray:
    """Compute Mean Prediction Interval Width per DOF.

    Args:
        intervals: Prediction intervals with shape ``(N, 6, 2)``. The last
            dimension stores lower and upper bounds.

    Returns:
        Mean interval width per DOF with shape ``(6,)``.

    Raises:
        ValueError: If input shape is invalid or interval bounds are ordered
            incorrectly.

    Units:
        DOF order is ``[X_cm, Y_cm, Z_cm, Roll_deg, Pitch_deg, Yaw_deg]``.
    """
    interval_array = _validate_intervals(intervals)
    widths = interval_array[:, :, 1] - interval_array[:, :, 0]
    return widths.mean(axis=0)


def interval_score(
    intervals: np.ndarray,
    ground_truths: np.ndarray,
    alpha: float,
) -> np.ndarray:
    """Compute interval score per DOF.

    Implements the Gneiting and Raftery interval score:

    ``(upper - lower) + (2 / alpha) * (lower - y) * I(y < lower)
    + (2 / alpha) * (y - upper) * I(y > upper)``.

    Args:
        intervals: Prediction intervals with shape ``(N, 6, 2)``. The last
            dimension stores lower and upper bounds.
        ground_truths: Ground-truth values with shape ``(N, 6)``.
        alpha: Significance level. For 90% coverage, use ``0.1``.

    Returns:
        Mean interval score per DOF with shape ``(6,)``.

    Raises:
        ValueError: If input shapes are invalid, interval bounds are ordered
            incorrectly, or ``alpha`` is outside ``(0, 1)``.

    Units:
        DOF order is ``[X_cm, Y_cm, Z_cm, Roll_deg, Pitch_deg, Yaw_deg]``.
    """
    if not 0.0 < alpha < 1.0:
        raise ValueError("alpha must be in the open interval (0, 1)")
    interval_array = _validate_intervals(intervals)
    ground_truth_array = _validate_ground_truths(ground_truths, interval_array.shape[0])
    lower = interval_array[:, :, 0]
    upper = interval_array[:, :, 1]
    widths = upper - lower
    below_penalty = (2.0 / alpha) * (lower - ground_truth_array) * (
        ground_truth_array < lower
    )
    above_penalty = (2.0 / alpha) * (ground_truth_array - upper) * (
        ground_truth_array > upper
    )
    return (widths + below_penalty + above_penalty).mean(axis=0)


def _validate_intervals(intervals: np.ndarray) -> np.ndarray:
    interval_array = np.asarray(intervals, dtype=float)
    if interval_array.ndim != 3 or interval_array.shape[1:] != (6, 2):
        raise ValueError("intervals must have shape (N, 6, 2)")
    if interval_array.shape[0] == 0:
        raise ValueError("intervals must contain at least one sample")
    if np.any(interval_array[:, :, 1] < interval_array[:, :, 0]):
        raise ValueError("interval upper bounds must be greater than or equal to lower bounds")
    return interval_array


def _validate_ground_truths(ground_truths: np.ndarray, sample_count: int) -> np.ndarray:
    ground_truth_array = np.asarray(ground_truths, dtype=float)
    if ground_truth_array.shape != (sample_count, 6):
        raise ValueError("ground_truths must have shape (N, 6) matching intervals")
    return ground_truth_array

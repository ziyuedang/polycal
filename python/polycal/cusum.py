"""CUSUM drift detection on EKF innovations."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class CUSUMConfig:
    """Configuration for CUSUM drift detector.

    Reference: Page, E.S. (1954) "Continuous Inspection Schemes."
    Biometrika 41(1/2):100-115.
    """

    # Reference value kappa for false-alarm control.
    #
    # Under no drift, |nu_std|^2 / dof ~ chi^2(dof)/dof with mean 1.0.
    # With kappa=1.0 the CUSUM increment has zero mean, making g_k a
    # reflected random walk with unpredictable false-alarm rate.
    #
    # Principled default: kappa = 1 + 0.85 * sqrt(2/dof)
    # For dof=6: kappa = 1 + 0.85 * sqrt(1/3) ≈ 1.5
    #
    # kappa=1.0 is INVALID for false-alarm-controlled operation.
    # Reference: Page (1954) Biometrika 41(1/2):100-115;
    #            Basseville & Nikiforov (1993) Ch. 2.
    kappa: float = 1.5
    # Detection threshold h. Alarm when g_k > h.
    threshold: float = 5.0
    # Degrees of freedom of the innovation vector.
    dof: int = 6


@dataclass
class CUSUMCalibration:
    """Result of empirical CUSUM kappa calibration.

    kappa: calibrated reference value
    mean: empirical mean of |nu_std|^2 / dof from calibration data
    std: empirical std of |nu_std|^2 / dof
    percentile: target percentile used (e.g. 0.80)
    n: number of calibration steps used
    """

    kappa: float
    mean: float
    std: float
    percentile: float
    n: int


def calibrate_kappa(
    no_drift_innovations: list[np.ndarray],
    no_drift_innovation_covs: list[np.ndarray],
    target_percentile: float = 0.80,
) -> CUSUMCalibration:
    """Estimate kappa from a held-out no-drift sequence.

    Computes |nu_std|^2 / dof for each step using the provided
    innovations and innovation covariances. Returns the
    target_percentile of the empirical distribution as kappa.

    This ensures E[increment] < 0 under no drift when the actual
    innovation distribution matches the calibration data.

    Note: this gives empirical operating characteristics, not a
    distribution-free guarantee. Conformal prediction (Cocheteux
    et al. arXiv:2501.06878) provides formal marginal coverage
    guarantees under exchangeability; this calibration provides
    empirical false-alarm control under stationarity.

    Args:
        no_drift_innovations: list of shape (6,) arrays, one per step
        no_drift_innovation_covs: list of shape (6,6) arrays, one per step
        target_percentile: percentile of empirical distribution to use
            as kappa. Default 0.80. Higher = more conservative
            (fewer false alarms, slower detection).

    Returns:
        CUSUMCalibration with calibrated kappa and diagnostics.

    Raises:
        ValueError: if lists are empty or have mismatched lengths.
    """
    if len(no_drift_innovations) == 0:
        raise ValueError("no_drift_innovations must not be empty")
    if len(no_drift_innovations) != len(no_drift_innovation_covs):
        raise ValueError(
            "no_drift_innovations and no_drift_innovation_covs "
            "must have the same length"
        )

    dof = len(no_drift_innovations[0])
    normalized = []
    for nu, S in zip(no_drift_innovations, no_drift_innovation_covs):
        S_inv = np.linalg.inv(S)
        mahal_sq = float(nu @ S_inv @ nu)
        normalized.append(mahal_sq / dof)

    arr = np.array(normalized)
    kappa = float(np.percentile(arr, target_percentile * 100))
    return CUSUMCalibration(
        kappa=kappa,
        mean=float(arr.mean()),
        std=float(arr.std()),
        percentile=target_percentile,
        n=len(arr),
    )


class CUSUMDetector:
    """CUSUM change-point detector on EKF innovations.

    Monitors the standardized innovation magnitude. Under no drift, the CUSUM
    statistic stays near zero. Under drift, it accumulates and triggers an
    alarm when it exceeds the configured threshold.
    """

    def __init__(
        self,
        config: CUSUMConfig | None = None,
        backend: str = "python",
    ) -> None:
        """Initialize the detector."""
        if config is None:
            config = CUSUMConfig()

        if backend == "cpp":
            try:
                from polycal import _polycal_cpp
            except ImportError as e:
                raise ImportError("C++ backend not available.") from e
            cpp_config = _polycal_cpp.CUSUMConfig()
            cpp_config.kappa = config.kappa
            cpp_config.threshold = config.threshold
            cpp_config.dof = config.dof
            self._backend = "cpp"
            self._cpp = _polycal_cpp.CUSUMDetector(cpp_config)
        elif backend == "python":
            self._backend = "python"
            self._cpp = None
            self.config = config
            self.g_ = 0.0
            self.step_ = 0
            self.alarm_step_: int | None = None
        else:
            raise ValueError(f"backend must be 'python' or 'cpp', got {backend!r}")

    def update(
        self,
        innovation: np.ndarray,
        innovation_cov: np.ndarray,
    ) -> tuple[bool, float]:
        """Update CUSUM with one innovation vector.

        Args:
            innovation: EKF innovation vector with shape ``(6,)``.
            innovation_cov: Innovation covariance with shape ``(6, 6)``.

        Returns:
            Tuple ``(alarm, g)`` with alarm state and current statistic.
        """
        if self._backend == "cpp":
            return self._cpp.update(innovation, innovation_cov)
        S_inv = np.linalg.inv(innovation_cov)
        mahal_sq = float(innovation @ S_inv @ innovation)
        normalized = mahal_sq / self.config.dof

        self.g_ = max(0.0, self.g_ + normalized - self.config.kappa)
        self.step_ += 1

        alarm = self.g_ > self.config.threshold
        if alarm and self.alarm_step_ is None:
            self.alarm_step_ = self.step_

        return alarm, self.g_

    def reset(self) -> None:
        """Reset CUSUM statistic. Call after recalibration."""
        if self._backend == "cpp":
            self._cpp.reset()
            return
        self.g_ = 0.0
        self.alarm_step_ = None

    @property
    def statistic(self) -> float:
        """Return the current CUSUM statistic."""
        if self._backend == "cpp":
            return self._cpp.statistic()
        return self.g_

    @property
    def steps_to_alarm(self) -> int | None:
        """Steps from start to first alarm, or ``None`` if no alarm yet."""
        if self._backend == "cpp":
            step = self._cpp.steps_to_alarm()
            return None if step < 0 else step
        return self.alarm_step_


def compute_innovation_cov(
    H: np.ndarray,
    P: np.ndarray,
    R: np.ndarray,
) -> np.ndarray:
    """Compute innovation covariance ``S = H P H.T + R``."""
    return H @ P @ H.T + R

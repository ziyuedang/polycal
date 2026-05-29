from __future__ import annotations

import numpy as np
import pytest

from polycal.cusum import (
    CUSUMConfig,
    CUSUMDetector,
    calibrate_kappa,
    compute_innovation_cov,
)


def test_cusum_no_drift_no_alarm() -> None:
    rng = np.random.default_rng(0)
    detector = CUSUMDetector(CUSUMConfig())
    innovation_cov = np.eye(6)
    alarms = 0

    for step in range(10_000):
        alarm, _ = detector.update(rng.normal(0.0, 1.0, size=6), innovation_cov)
        if step < 1000:
            assert not alarm
        if alarm:
            alarms += 1
            detector.reset()

    assert alarms < 5


def test_cusum_step_drift_triggers_alarm() -> None:
    rng = np.random.default_rng(1)
    detector = CUSUMDetector(CUSUMConfig(threshold=5.0, dof=6))
    innovation_cov = np.eye(6)

    for _ in range(200):
        detector.update(rng.normal(0.0, 1.0, size=6), innovation_cov)

    drift = np.array([0.0, 0.0, 0.0, 3.0, 0.0, 0.0])
    for step in range(1, 51):
        alarm, _ = detector.update(rng.normal(0.0, 1.0, size=6) + drift, innovation_cov)
        if alarm:
            assert step <= 50
            return

    raise AssertionError("CUSUM did not alarm within 50 drift steps")


def test_cusum_resets_after_alarm() -> None:
    detector = CUSUMDetector(CUSUMConfig(threshold=1.0, dof=6))
    innovation_cov = np.eye(6)
    alarm, _ = detector.update(np.array([5.0, 0.0, 0.0, 0.0, 0.0, 0.0]), innovation_cov)
    assert alarm

    detector.reset()
    assert detector.statistic == 0.0
    assert detector.steps_to_alarm is None
    alarm, _ = detector.update(np.zeros(6), innovation_cov)
    assert not alarm


def test_cusum_statistic_increases_under_drift() -> None:
    detector = CUSUMDetector(CUSUMConfig(threshold=100.0, dof=6))
    innovation_cov = np.eye(6)
    values = []

    for _ in range(5):
        _, statistic = detector.update(np.array([6.0, 0.0, 0.0, 0.0, 0.0, 0.0]), innovation_cov)
        values.append(statistic)

    assert all(after > before for before, after in zip(values, values[1:]))


def test_cusum_statistic_floors_at_zero() -> None:
    detector = CUSUMDetector(CUSUMConfig(threshold=5.0, dof=6))
    innovation_cov = np.eye(6)

    for _ in range(10):
        _, statistic = detector.update(np.full(6, 1e-6), innovation_cov)
        assert statistic == 0.0


def test_compute_innovation_cov() -> None:
    H = np.eye(6)
    P = 2.0 * np.eye(6)
    R = np.eye(6)
    S = compute_innovation_cov(H, P, R)
    assert np.allclose(S, 3.0 * np.eye(6))


def test_calibrate_kappa_basic() -> None:
    rng = np.random.default_rng(42)
    innovations = [rng.standard_normal(6) for _ in range(2000)]
    covs = [np.eye(6)] * 2000

    result = calibrate_kappa(innovations, covs, target_percentile=0.80)

    assert 1.2 < result.kappa < 1.7
    assert result.n == 2000
    assert result.mean > 0
    assert result.std > 0
    assert result.percentile == 0.80


def test_calibrate_kappa_empty_raises() -> None:
    with pytest.raises(ValueError):
        calibrate_kappa([], [], target_percentile=0.80)


def test_calibrate_kappa_mismatched_raises() -> None:
    innovations = [np.zeros(6)]
    covs = []
    with pytest.raises(ValueError):
        calibrate_kappa(innovations, covs, target_percentile=0.80)

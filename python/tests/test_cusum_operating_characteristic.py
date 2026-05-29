from __future__ import annotations

from pathlib import Path

import numpy as np

from polycal.cusum import CUSUMConfig, CUSUMDetector, calibrate_kappa
from polycal.ekf import ExtrinsicEKF
from polycal.lie_utils import se3_exp
from polycal.synthetic import (
    LidarModel,
    OdometryConfig,
    SceneConfig,
    StaticDrift,
    SyntheticConfig,
    VehicleTrajectoryConfig,
    generate,
)


def _analytic_P_init() -> np.ndarray:
    return np.diag([1e-4] * 6)


def _analytic_Q() -> np.ndarray:
    return np.diag([1e-8] * 3 + [1e-7] * 3)


def _analytic_R() -> np.ndarray:
    return np.diag([2e-4] * 3 + [2e-6] * 3)


def _odometry_config() -> OdometryConfig:
    return OdometryConfig(
        translation_noise_std=0.01,
        rotation_noise_std=0.001,
    )


def _empty_scene_config(
    *,
    seed: int,
    duration_s: float,
    rate_hz: float,
    drift_profile: StaticDrift,
) -> SyntheticConfig:
    return SyntheticConfig(
        duration_s=duration_s,
        rate_hz=rate_hz,
        seed=seed,
        lidar=LidarModel(n_beams=1, horizontal_resolution_deg=360.0),
        scene=SceneConfig(planes=[], lines=[], landmarks=[]),
        drift_profile=drift_profile,
        odometry_camera=_odometry_config(),
        odometry_lidar=_odometry_config(),
        vehicle_trajectory=VehicleTrajectoryConfig(trajectory_type="sinusoidal_3d"),
    )


def _run_cusum_normalized(
    normalized: np.ndarray,
    threshold: float,
    kappa: float,
) -> np.ndarray:
    g = np.zeros(normalized.shape[1], dtype=float)
    first_alarm = np.zeros(normalized.shape[1], dtype=int)
    active = np.ones(normalized.shape[1], dtype=bool)

    for step, values in enumerate(normalized, start=1):
        indices = np.flatnonzero(active)
        g[indices] = np.maximum(0.0, g[indices] + values[indices] - kappa)
        alarm = g[indices] > threshold
        if np.any(alarm):
            alarm_indices = indices[alarm]
            first_alarm[alarm_indices] = step
            active[alarm_indices] = False
        if not np.any(active):
            break

    return first_alarm


def run_arl_experiment(
    threshold: float,
    n_trials: int = 500,
    max_steps: int = 50000,
    seed: int = 0,
) -> float:
    """Estimate ARL_0 under no drift for a given threshold.

    The empirical curve uses kappa=1.5 so the no-drift random walk has
    negative mean increment. With kappa=1.0, it false-alarms quickly because
    the expected increment is zero.
    """
    rng = np.random.default_rng(seed)
    normalized = rng.chisquare(6, size=(max_steps, n_trials)) / 6.0
    first_alarm = _run_cusum_normalized(normalized, threshold, kappa=1.5)
    first_alarm[first_alarm == 0] = max_steps
    return float(np.mean(first_alarm))


def run_detection_delay_experiment(
    threshold: float,
    drift_magnitude: float,
    n_trials: int = 500,
    warmup_steps: int = 100,
    seed: int = 0,
) -> float:
    """Estimate mean detection delay for a step drift of given magnitude."""
    rng = np.random.default_rng(seed)
    max_delay_steps = 200
    normalized = rng.chisquare(
        6,
        size=(warmup_steps + max_delay_steps, n_trials),
    ) / 6.0
    normalized[warmup_steps:] += drift_magnitude
    first_alarm = _run_cusum_normalized(normalized, threshold, kappa=1.5)
    first_alarm[first_alarm == 0] = warmup_steps + max_delay_steps
    delays = np.maximum(0, first_alarm - warmup_steps)
    return float(np.mean(delays))


def test_operating_characteristic_curve() -> None:
    """Generate a CUSUM operating-characteristic table."""
    rows = []
    for threshold in [1, 2, 3, 4, 5, 6]:
        arl_0 = run_arl_experiment(float(threshold))
        delay = run_detection_delay_experiment(float(threshold), drift_magnitude=1.0)
        rows.append((threshold, arl_0, delay))

    print("h    ARL_0      Detection_delay")
    for threshold, arl_0, delay in rows:
        print(f"{threshold:<4d} {arl_0:10.1f} {delay:16.1f}")

    output_path = Path("data/cusum_operating_characteristic.csv")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(
        output_path,
        np.asarray(rows, dtype=float),
        delimiter=",",
        header="threshold,arl_0,detection_delay",
        comments="",
    )

    h5 = rows[4]
    assert h5[1] > 5000
    assert h5[2] < 20


def test_ekf_cusum_integration() -> None:
    """Run EKF + CUSUM on a static phase followed by a step drift."""
    phase1 = generate(
        _empty_scene_config(
            seed=13,
            duration_s=20.1,
            rate_hz=10.0,
            drift_profile=StaticDrift(np.eye(4)),
        )
    )
    step_drift = se3_exp(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.1]))
    phase2 = generate(
        _empty_scene_config(
            seed=100,
            duration_s=20.1,
            rate_hz=10.0,
            drift_profile=StaticDrift(step_drift),
        )
    )

    ekf = ExtrinsicEKF(np.eye(4), _analytic_P_init(), _analytic_Q())
    detector = CUSUMDetector(CUSUMConfig(threshold=5.0, kappa=1.0, dof=6))
    R = _analytic_R()
    dt = 1.0 / phase1.config.rate_hz
    statistic_trace = []

    false_alarms = 0
    for T_cam_odom, T_lidar_odom in zip(
        phase1.camera_odometry,
        phase1.lidar_odometry,
    ):
        ekf.predict(dt)
        innovation, innovation_cov = ekf.update_with_cusum(T_cam_odom, T_lidar_odom, R)
        alarm, statistic = detector.update(innovation, innovation_cov)
        statistic_trace.append(statistic)
        false_alarms += int(alarm)
        assert not alarm

    steps_to_detection = None
    for step, (T_cam_odom, T_lidar_odom) in enumerate(
        zip(phase2.camera_odometry, phase2.lidar_odometry),
        start=1,
    ):
        ekf.predict(dt)
        innovation, innovation_cov = ekf.update_with_cusum(T_cam_odom, T_lidar_odom, R)
        alarm, statistic = detector.update(innovation, innovation_cov)
        statistic_trace.append(statistic)
        if alarm:
            steps_to_detection = step
            break

    trace = np.asarray(statistic_trace)
    print(f"False alarms in phase 1: {false_alarms}")
    print(f"Steps to detection in phase 2: {steps_to_detection}")
    print(
        "CUSUM statistic trace: "
        f"min={np.min(trace):.3f}, mean={np.mean(trace):.3f}, max={np.max(trace):.3f}"
    )

    assert false_alarms == 0
    assert steps_to_detection is not None
    assert steps_to_detection <= 30


def test_calibrate_kappa_reduces_false_alarms() -> None:
    """Demonstrate that calibrated kappa reduces false alarms vs kappa=1.0."""
    rng = np.random.default_rng(0)

    cal_innovations = [rng.standard_normal(6) for _ in range(1000)]
    cal_covs = [np.eye(6)] * 1000
    calibration = calibrate_kappa(cal_innovations, cal_covs)

    test_innovations = [rng.standard_normal(6) for _ in range(5000)]

    det_cal = CUSUMDetector(CUSUMConfig(kappa=calibration.kappa))
    alarms_cal = sum(det_cal.update(nu, np.eye(6))[0] for nu in test_innovations)

    det_10 = CUSUMDetector(CUSUMConfig(kappa=1.0))
    alarms_10 = sum(det_10.update(nu, np.eye(6))[0] for nu in test_innovations)

    print(f"Alarms with kappa=1.0: {alarms_10}")
    print(f"Alarms with kappa={calibration.kappa:.3f}: {alarms_cal}")
    assert alarms_cal < alarms_10

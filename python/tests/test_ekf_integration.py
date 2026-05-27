from __future__ import annotations

import numpy as np
from scipy.stats import chi2

from polycal.ekf import ExtrinsicEKF
from polycal.lie_utils import se3_exp, se3_log
from polycal.synthetic import (
    LidarModel,
    LinearDrift,
    OdometryConfig,
    SceneConfig,
    StaticDrift,
    SyntheticConfig,
    VehicleTrajectoryConfig,
    generate,
)


def _scaled_identity(scale: float) -> np.ndarray:
    return scale * np.eye(6)


def _error_tangent(T_true: np.ndarray, T_est: np.ndarray) -> np.ndarray:
    return se3_log(np.linalg.inv(T_true) @ T_est)


def _empty_scene_config(
    *,
    seed: int,
    duration_s: float,
    rate_hz: float,
    drift_profile: LinearDrift | StaticDrift,
    odometry: OdometryConfig | None = None,
    vehicle_trajectory: VehicleTrajectoryConfig | None = None,
) -> SyntheticConfig:
    return SyntheticConfig(
        duration_s=duration_s,
        rate_hz=rate_hz,
        seed=seed,
        lidar=LidarModel(n_beams=1, horizontal_resolution_deg=360.0),
        scene=SceneConfig(planes=[], lines=[], landmarks=[]),
        drift_profile=drift_profile,
        odometry_camera=odometry or OdometryConfig(),
        odometry_lidar=odometry or OdometryConfig(),
        vehicle_trajectory=vehicle_trajectory or VehicleTrajectoryConfig(),
    )


def test_ekf_converges_on_static_extrinsic() -> None:
    """EKF converges from identity to a small static LiDAR-camera transform."""
    true_tangent = np.zeros(6)
    true_tangent[0] = 0.05
    true_tangent[5] = 0.1
    T_lc_true = se3_exp(true_tangent)
    dataset = generate(
        _empty_scene_config(
            seed=2,
            duration_s=20.1,
            rate_hz=10.0,
            drift_profile=StaticDrift(T_lc_true),
            odometry=OdometryConfig(
                translation_noise_std=0.0,
                rotation_noise_std=0.0,
            ),
        )
    )
    ekf = ExtrinsicEKF(np.eye(4), _scaled_identity(1.0), _scaled_identity(1e-8))
    R = _scaled_identity(1e-4)
    trace_initial = float(np.trace(ekf.P))

    for T_cam_odom, T_lidar_odom in zip(
        dataset.camera_odometry,
        dataset.lidar_odometry,
    ):
        ekf.update(T_cam_odom, T_lidar_odom, R)

    error = _error_tangent(T_lc_true, ekf.T_lc)
    assert np.linalg.norm(error[:3]) < 0.01
    assert np.linalg.norm(error[3:]) < 0.01
    assert float(np.trace(ekf.P)) < 0.5 * trace_initial


def test_ekf_tracks_linear_drift() -> None:
    """EKF tracks a slowly drifting extrinsic using noisy generated odometry."""
    rotation_noise_std = 0.001
    drift = LinearDrift(
        T_start=np.eye(4),
        angular_velocity_rad_s=np.array([0.0, 0.0, 0.001]),
        linear_velocity_m_s=np.zeros(3),
    )
    dataset = generate(
        _empty_scene_config(
            seed=1,
            duration_s=60.0,
            rate_hz=10.0,
            drift_profile=drift,
            odometry=OdometryConfig(
                translation_noise_std=0.0,
                rotation_noise_std=rotation_noise_std,
            ),
            vehicle_trajectory=VehicleTrajectoryConfig(
                trajectory_type="sinusoidal_3d"
            ),
        )
    )
    ekf = ExtrinsicEKF(dataset.gt_extrinsics[0], _scaled_identity(0.1), _scaled_identity(1e-3))
    R = _scaled_identity(1e-3)
    dt = 1.0 / dataset.config.rate_hz
    final_translation_error = 0.0
    final_rotation_error = 0.0

    for index in range(dataset.gt_extrinsics.shape[0] - 1):
        T_lc_step = dataset.gt_extrinsics[index]
        ekf.predict(dt)
        ekf.update(dataset.camera_odometry[index], dataset.lidar_odometry[index], R)
        error = _error_tangent(T_lc_step, ekf.T_lc)
        final_translation_error = float(np.linalg.norm(error[:3]))
        final_rotation_error = float(np.linalg.norm(error[3:]))
        assert final_rotation_error < 0.01
        assert final_translation_error < 0.05

    print(
        "Final drift tracking error: "
        f"translation={final_translation_error:.6f} m, "
        f"rotation={final_rotation_error:.6f} rad"
    )


def test_ekf_covariance_calibration_sanity() -> None:
    """Estimate Phase 1 empirical coverage of the 95% covariance ellipsoid."""
    rng = np.random.default_rng(4)
    trials = 500
    covered = 0
    threshold = float(chi2.ppf(0.95, df=6))
    T_lc_true = se3_exp(np.array([0.03, -0.02, 0.01, 0.04, -0.03, 0.02]))

    for trial in range(trials):
        dataset = generate(
            _empty_scene_config(
                seed=trial,
                duration_s=5.1,
                rate_hz=10.0,
                drift_profile=StaticDrift(T_lc_true),
                odometry=OdometryConfig(
                    translation_noise_std=0.0,
                    rotation_noise_std=0.0,
                ),
                vehicle_trajectory=VehicleTrajectoryConfig(
                    trajectory_type="sinusoidal_3d"
                ),
            )
        )
        perturbation = rng.normal(0.0, 0.01, size=6)
        T_lc_init = T_lc_true @ se3_exp(perturbation)
        ekf = ExtrinsicEKF(T_lc_init, _scaled_identity(0.02), _scaled_identity(1e-8))
        R = _scaled_identity(1e-4)

        for T_cam_odom, T_lidar_odom in zip(
            dataset.camera_odometry,
            dataset.lidar_odometry,
        ):
            ekf.update(T_cam_odom, T_lidar_odom, R)

        error = _error_tangent(T_lc_true, ekf.T_lc)
        covariance = 0.5 * (ekf.P + ekf.P.T)
        mahalanobis_sq = float(error.T @ np.linalg.solve(covariance, error))
        covered += mahalanobis_sq <= threshold

    coverage = covered / trials
    print(f"Phase 1 EKF 95% ellipsoid empirical coverage: {coverage:.3f}")
    assert 0.85 <= coverage <= 1.0

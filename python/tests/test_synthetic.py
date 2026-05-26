from __future__ import annotations

from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from polycal import (
    CameraModel,
    Landmark,
    LinearDrift,
    LidarModel,
    SceneConfig,
    StaticDrift,
    SyntheticConfig,
    SyntheticDataset,
    generate,
)
from polycal.synthetic import OdometryConfig, VehicleTrajectoryConfig


def _small_config(seed: int = 7) -> SyntheticConfig:
    return SyntheticConfig(
        duration_s=1.0,
        rate_hz=5.0,
        seed=seed,
        lidar=LidarModel(horizontal_resolution_deg=5.0),
    )


def _odometry_only_config(
    *,
    seed: int = 0,
    duration_s: float = 1000.0,
    odometry: OdometryConfig | None = None,
    drift_profile: LinearDrift | StaticDrift | None = None,
    vehicle_trajectory: VehicleTrajectoryConfig | None = None,
) -> SyntheticConfig:
    return SyntheticConfig(
        duration_s=duration_s,
        rate_hz=1.0,
        seed=seed,
        lidar=LidarModel(n_beams=1, horizontal_resolution_deg=360.0),
        scene=SceneConfig(planes=[], lines=[], landmarks=[]),
        drift_profile=drift_profile or StaticDrift(np.eye(4)),
        odometry_camera=odometry or OdometryConfig(),
        odometry_lidar=odometry or OdometryConfig(),
        vehicle_trajectory=vehicle_trajectory or VehicleTrajectoryConfig(),
    )


def _relative_noise(
    T_gt: np.ndarray,
    T_noisy: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    relative = np.linalg.inv(T_gt) @ T_noisy
    translation = relative[:3, 3]
    rotation = Rotation.from_matrix(relative[:3, :3]).as_rotvec()
    return translation, rotation


def test_static_drift_produces_constant_ground_truth() -> None:
    T = np.eye(4)
    T[:3, 3] = np.array([0.1, -0.2, 0.3])
    dataset = generate(
        SyntheticConfig(
            duration_s=1.0,
            rate_hz=5.0,
            seed=1,
            lidar=LidarModel(horizontal_resolution_deg=10.0),
            drift_profile=StaticDrift(T),
        )
    )

    for gt in dataset.gt_extrinsics:
        np.testing.assert_allclose(gt, T)


def test_linear_drift_accumulates_over_time() -> None:
    pitch_rate = np.deg2rad(0.5) / 30.0
    drift = LinearDrift(
        T_start=np.eye(4),
        angular_velocity_rad_s=np.array([0.0, pitch_rate, 0.0]),
        linear_velocity_m_s=np.zeros(3),
    )

    T_30 = drift(30.0)
    expected = Rotation.from_euler("y", 0.5, degrees=True).as_matrix()
    np.testing.assert_allclose(T_30[:3, :3], expected, atol=1e-12)


def test_seed_determinism(tmp_path: Path) -> None:
    first = generate(_small_config(seed=9))
    second = generate(_small_config(seed=9))

    np.testing.assert_array_equal(first.timestamps, second.timestamps)
    np.testing.assert_array_equal(first.gt_extrinsics, second.gt_extrinsics)
    for left, right in zip(first.lidar_sweeps, second.lidar_sweeps):
        np.testing.assert_array_equal(left, right)
    for left, right in zip(first.image_features, second.image_features):
        np.testing.assert_array_equal(left, right)

    first_path = tmp_path / "first.npz"
    second_path = tmp_path / "second.npz"
    first.save(first_path)
    second.save(second_path)
    assert first_path.read_bytes() == second_path.read_bytes()

    loaded = SyntheticDataset.load(first_path)
    np.testing.assert_array_equal(first.timestamps, loaded.timestamps)
    np.testing.assert_array_equal(first.gt_extrinsics, loaded.gt_extrinsics)
    for left, right in zip(first.lidar_sweeps, loaded.lidar_sweeps):
        np.testing.assert_array_equal(left, right)


def test_lidar_sweep_size() -> None:
    config = _small_config()
    ray_count = config.lidar.sweep_directions().shape[0]
    assert ray_count == 64 * 72
    dataset = generate(config)

    for sweep in dataset.lidar_sweeps:
        assert sweep.shape[1] == 3
        assert sweep.shape[0] <= ray_count


def test_default_lidar_ray_count() -> None:
    directions = LidarModel().sweep_directions()
    assert directions.shape == (64 * 1800, 3)
    np.testing.assert_allclose(np.linalg.norm(directions, axis=1), 1.0)


def test_image_feature_counts_reasonable() -> None:
    dataset = generate(_small_config())

    for features in dataset.image_features:
        assert 5 <= features.shape[0] <= 50
        assert np.all(features[:, 0] >= 0.0)
        assert np.all(features[:, 0] < dataset.config.camera.width)
        assert np.all(features[:, 1] >= 0.0)
        assert np.all(features[:, 1] < dataset.config.camera.height)


def test_correspondences_consistent() -> None:
    config = _small_config()
    dataset = generate(config)
    scene = dataset.config.scene

    for frame_idx, frame_corrs in enumerate(dataset.correspondences):
        T_lc = dataset.gt_extrinsics[frame_idx]
        for corr in frame_corrs:
            landmark = scene.landmarks[corr.landmark_id].position
            point_cam = (T_lc[:3, :3] @ landmark) + T_lc[:3, 3]
            projected = dataset.config.camera.project(point_cam[None, :])[0]
            assert np.linalg.norm(corr.pixel - projected) < 5.0 * config.camera.pixel_noise_std
            assert np.linalg.norm(corr.lidar_point - landmark) < 5.0 * config.lidar.range_noise_std


def test_save_load_roundtrip(tmp_path: Path) -> None:
    path = tmp_path / "synthetic.npz"
    dataset = generate(_small_config(seed=12))
    dataset.save(path)
    loaded = SyntheticDataset.load(path)

    np.testing.assert_array_equal(dataset.timestamps, loaded.timestamps)
    np.testing.assert_array_equal(dataset.gt_extrinsics, loaded.gt_extrinsics)
    for left, right in zip(dataset.image_features, loaded.image_features):
        np.testing.assert_array_equal(left, right)
    for left_frame, right_frame in zip(dataset.correspondences, loaded.correspondences):
        assert len(left_frame) == len(right_frame)
        for left, right in zip(left_frame, right_frame):
            np.testing.assert_array_equal(left.lidar_point, right.lidar_point)
            np.testing.assert_array_equal(left.pixel, right.pixel)
            assert left.landmark_id == right.landmark_id


def test_pitch_drift_at_thirty_seconds() -> None:
    drift = LinearDrift(
        T_start=np.eye(4),
        angular_velocity_rad_s=np.array([0.0, np.deg2rad(0.5) / 30.0, 0.0]),
        linear_velocity_m_s=np.zeros(3),
    )
    dataset = generate(
        SyntheticConfig(
            duration_s=31.0,
            rate_hz=1.0,
            seed=3,
            lidar=LidarModel(horizontal_resolution_deg=10.0),
            drift_profile=drift,
        )
    )

    frame = int(np.flatnonzero(np.isclose(dataset.timestamps, 30.0))[0])
    expected = Rotation.from_euler("y", 0.5, degrees=True).as_matrix()
    np.testing.assert_allclose(dataset.gt_extrinsics[frame, :3, :3], expected, atol=1e-12)


def test_custom_single_landmark_scene() -> None:
    scene = SceneConfig(
        planes=[],
        lines=[],
        landmarks=[Landmark(np.array([0.0, 0.0, 10.0]))],
    )
    dataset = generate(
        SyntheticConfig(
            duration_s=1.0,
            rate_hz=1.0,
            seed=4,
            camera=CameraModel(pixel_noise_std=0.0),
            lidar=LidarModel(horizontal_resolution_deg=30.0, range_noise_std=0.0),
            scene=scene,
        )
    )

    assert dataset.image_features[0].shape == (1, 2)
    np.testing.assert_allclose(dataset.image_features[0][0], np.array([320.0, 240.0]))


def test_odometry_shapes() -> None:
    dataset = generate(_small_config())
    step_count = dataset.timestamps.shape[0] - 1

    assert dataset.camera_odometry.shape == (step_count, 4, 4)
    assert dataset.lidar_odometry.shape == (step_count, 4, 4)
    assert dataset.camera_odometry_gt.shape == (step_count, 4, 4)
    assert dataset.lidar_odometry_gt.shape == (step_count, 4, 4)


def test_odometry_gt_consistent_with_vehicle_trajectory() -> None:
    T_lc = np.eye(4)
    T_lc[:3, 3] = np.array([0.1, -0.2, 0.3])
    dataset = generate(
        SyntheticConfig(
            duration_s=1.0,
            rate_hz=5.0,
            seed=5,
            lidar=LidarModel(horizontal_resolution_deg=30.0),
            drift_profile=StaticDrift(T_lc),
            vehicle_trajectory=VehicleTrajectoryConfig(trajectory_type="figure8"),
        )
    )

    for camera_odom, lidar_odom in zip(
        dataset.camera_odometry_gt,
        dataset.lidar_odometry_gt,
    ):
        np.testing.assert_allclose(
            camera_odom,
            T_lc @ lidar_odom @ np.linalg.inv(T_lc),
            atol=1e-10,
        )


def test_hand_eye_constraint_satisfied() -> None:
    T_lc = np.eye(4)
    T_lc[:3, 3] = np.array([0.1, -0.2, 0.3])
    trajectory_types = ["figure8", "random_walk", "straight"]

    for trajectory_type in trajectory_types:
        dataset = generate(
            SyntheticConfig(
                duration_s=1.0,
                rate_hz=5.0,
                seed=5,
                lidar=LidarModel(horizontal_resolution_deg=30.0),
                drift_profile=StaticDrift(T_lc),
                vehicle_trajectory=VehicleTrajectoryConfig(
                    trajectory_type=trajectory_type
                ),
            )
        )

        for camera_odom, lidar_odom in zip(
            dataset.camera_odometry_gt,
            dataset.lidar_odometry_gt,
        ):
            np.testing.assert_allclose(
                camera_odom,
                T_lc @ lidar_odom @ np.linalg.inv(T_lc),
                atol=1e-10,
            )


def test_vehicle_poses_shape() -> None:
    dataset = generate(
        SyntheticConfig(
            duration_s=1.0,
            rate_hz=5.0,
            seed=5,
            lidar=LidarModel(horizontal_resolution_deg=30.0),
        )
    )
    step_count = dataset.timestamps.shape[0] - 1

    assert dataset.vehicle_poses.shape == (dataset.timestamps.shape[0], 4, 4)
    assert dataset.vehicle_odometry_gt.shape == (step_count, 4, 4)


def test_figure8_trajectory_has_rotation() -> None:
    dataset = generate(
        _odometry_only_config(
            vehicle_trajectory=VehicleTrajectoryConfig(trajectory_type="figure8")
        )
    )
    angles = [
        np.linalg.norm(Rotation.from_matrix(T[:3, :3]).as_rotvec())
        for T in dataset.vehicle_odometry_gt
    ]

    assert float(np.mean(angles)) > 0.01


def test_straight_trajectory_no_rotation() -> None:
    dataset = generate(
        _odometry_only_config(
            vehicle_trajectory=VehicleTrajectoryConfig(trajectory_type="straight")
        )
    )

    for T_vehicle_odom in dataset.vehicle_odometry_gt:
        np.testing.assert_allclose(T_vehicle_odom[:3, :3], np.eye(3), atol=1e-10)


def test_noisy_odometry_noise_level() -> None:
    translation_std = 0.02
    rotation_std = 0.003
    config = _odometry_only_config(
        odometry=OdometryConfig(
            translation_noise_std=translation_std,
            rotation_noise_std=rotation_std,
        )
    )
    dataset = generate(config)
    translation_errors = []
    rotation_errors = []

    for T_gt, T_noisy in zip(dataset.camera_odometry_gt, dataset.camera_odometry):
        translation, rotation = _relative_noise(T_gt, T_noisy)
        translation_errors.append(translation)
        rotation_errors.append(rotation)

    translation_empirical = float(np.std(np.vstack(translation_errors)))
    rotation_empirical = float(np.std(np.vstack(rotation_errors)))
    assert abs(translation_empirical - translation_std) / translation_std < 0.05
    assert abs(rotation_empirical - rotation_std) / rotation_std < 0.05


def test_degeneracy_injection() -> None:
    config = _odometry_only_config(
        seed=0,
        odometry=OdometryConfig(
            translation_noise_std=0.01,
            rotation_noise_std=0.001,
            degeneracy_fraction=0.5,
        ),
    )
    dataset = generate(config)
    rotation_norms = []

    for T_gt, T_noisy in zip(dataset.camera_odometry_gt, dataset.camera_odometry):
        _, rotation = _relative_noise(T_gt, T_noisy)
        rotation_norms.append(np.linalg.norm(rotation))

    near_zero_fraction = float(np.mean(np.array(rotation_norms) < 0.01))
    assert 0.4 < near_zero_fraction < 0.6


def test_odometry_save_load_roundtrip(tmp_path: Path) -> None:
    path = tmp_path / "synthetic_odometry.npz"
    dataset = generate(_small_config(seed=13))
    dataset.save(path)
    loaded = SyntheticDataset.load(path)

    np.testing.assert_array_equal(dataset.camera_odometry, loaded.camera_odometry)
    np.testing.assert_array_equal(dataset.lidar_odometry, loaded.lidar_odometry)
    np.testing.assert_array_equal(dataset.camera_odometry_gt, loaded.camera_odometry_gt)
    np.testing.assert_array_equal(dataset.lidar_odometry_gt, loaded.lidar_odometry_gt)
    np.testing.assert_array_equal(dataset.vehicle_poses, loaded.vehicle_poses)
    np.testing.assert_array_equal(dataset.vehicle_odometry_gt, loaded.vehicle_odometry_gt)

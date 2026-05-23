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


def _small_config(seed: int = 7) -> SyntheticConfig:
    return SyntheticConfig(
        duration_s=1.0,
        rate_hz=5.0,
        seed=seed,
        lidar=LidarModel(horizontal_resolution_deg=5.0),
    )


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

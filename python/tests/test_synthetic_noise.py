from __future__ import annotations

import numpy as np

from polycal import CameraModel, Landmark, LidarModel, SceneConfig, SyntheticConfig, generate


def test_pixel_noise_matches_config() -> None:
    pixel_noise_std = 1.3
    scene = SceneConfig(
        planes=[],
        lines=[],
        landmarks=[Landmark(np.array([0.0, 0.0, 10.0]))],
    )
    dataset = generate(
        SyntheticConfig(
            duration_s=1000.0,
            rate_hz=1.0,
            seed=22,
            camera=CameraModel(pixel_noise_std=pixel_noise_std),
            lidar=LidarModel(horizontal_resolution_deg=30.0),
            scene=scene,
        )
    )

    pixels = np.array([features[0] for features in dataset.image_features])
    errors = pixels - np.array([320.0, 240.0])
    empirical = errors.std(axis=0, ddof=1)
    np.testing.assert_allclose(empirical, pixel_noise_std, rtol=0.05)


def test_range_noise_matches_config() -> None:
    range_noise_std = 0.04
    scene = SceneConfig(
        planes=[],
        lines=[],
        landmarks=[Landmark(np.array([0.0, 0.0, 10.0]))],
    )
    dataset = generate(
        SyntheticConfig(
            duration_s=1000.0,
            rate_hz=1.0,
            seed=23,
            camera=CameraModel(pixel_noise_std=0.0),
            lidar=LidarModel(horizontal_resolution_deg=30.0, range_noise_std=range_noise_std),
            scene=scene,
        )
    )

    ranges = np.array(
        [frame_corrs[0].lidar_point[2] for frame_corrs in dataset.correspondences]
    )
    errors = ranges - 10.0
    np.testing.assert_allclose(errors.std(ddof=1), range_noise_std, rtol=0.05)

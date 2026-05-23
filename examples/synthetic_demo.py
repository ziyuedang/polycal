"""Generate and visualize a default synthetic polycal dataset."""

from __future__ import annotations

import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path("data") / ".matplotlib"))
os.environ.setdefault("MPLBACKEND", "Agg")

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

from polycal import LinearDrift, SyntheticConfig, generate


def main() -> None:
    """Generate the demo sequence, save it, and write a frame visualization."""
    drift = LinearDrift(
        T_start=np.eye(4),
        angular_velocity_rad_s=np.deg2rad(np.array([0.0, 0.5 / 60.0, 0.2 / 60.0])),
        linear_velocity_m_s=np.array([0.001, 0.0, 0.0]),
    )
    config = SyntheticConfig(drift_profile=drift)
    dataset = generate(config)

    output_dir = Path("data")
    output_dir.mkdir(parents=True, exist_ok=True)
    dataset_path = output_dir / "synthetic_demo.npz"
    dataset.save(dataset_path)

    frame_idx = 300
    T_lc = dataset.gt_extrinsics[frame_idx]
    points_lidar = dataset.lidar_sweeps[frame_idx]
    points_cam = (T_lc[:3, :3] @ points_lidar.T).T + T_lc[:3, 3]
    in_front = points_cam[:, 2] > 0.0
    pixels = dataset.config.camera.project(points_cam[in_front])
    depths = points_cam[in_front, 2]
    in_image = (
        (pixels[:, 0] >= 0.0)
        & (pixels[:, 0] < dataset.config.camera.width)
        & (pixels[:, 1] >= 0.0)
        & (pixels[:, 1] < dataset.config.camera.height)
    )

    fig, axis = plt.subplots(figsize=(8, 6), dpi=120)
    axis.set_facecolor("black")
    axis.set_xlim(0, dataset.config.camera.width)
    axis.set_ylim(dataset.config.camera.height, 0)
    axis.set_xlabel("u [px]")
    axis.set_ylabel("v [px]")
    axis.set_title("Synthetic frame 300")
    scatter = axis.scatter(
        pixels[in_image, 0],
        pixels[in_image, 1],
        c=depths[in_image],
        s=1.0,
        cmap="viridis",
        alpha=0.8,
    )
    features = dataset.image_features[frame_idx]
    axis.scatter(features[:, 0], features[:, 1], c="red", s=18.0, marker="x")
    fig.colorbar(scatter, ax=axis, label="depth [m]")
    fig.tight_layout()
    fig.savefig(output_dir / "synthetic_demo_frame300.png")
    plt.close(fig)

    total_lidar_returns = sum(sweep.shape[0] for sweep in dataset.lidar_sweeps)
    total_image_features = sum(features.shape[0] for features in dataset.image_features)
    final_rotation = dataset.gt_extrinsics[-1, :3, :3]
    drift_angle_deg = np.rad2deg(np.linalg.norm(Rotation.from_matrix(final_rotation).as_rotvec()))
    drift_translation_m = np.linalg.norm(dataset.gt_extrinsics[-1, :3, 3])
    print(f"frames: {dataset.timestamps.shape[0]}")
    print(f"total_lidar_returns: {total_lidar_returns}")
    print(f"total_image_features: {total_image_features}")
    print(f"drift_rotation_deg: {drift_angle_deg:.6f}")
    print(f"drift_translation_m: {drift_translation_m:.6f}")


if __name__ == "__main__":
    main()

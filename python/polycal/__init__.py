"""polycal — online LiDAR-camera extrinsic calibration with calibrated uncertainty."""

from __future__ import annotations

from polycal.synthetic import (
    CameraModel,
    LidarModel,
    Plane,
    LineSegment,
    Landmark,
    SceneConfig,
    StaticDrift,
    LinearDrift,
    SyntheticConfig,
    SyntheticDataset,
    Correspondence,
    generate,
)

__version__ = "0.1.0a1"

__all__ = [
    "CameraModel",
    "LidarModel",
    "Plane",
    "LineSegment",
    "Landmark",
    "SceneConfig",
    "StaticDrift",
    "LinearDrift",
    "SyntheticConfig",
    "SyntheticDataset",
    "Correspondence",
    "generate",
    "__version__",
]

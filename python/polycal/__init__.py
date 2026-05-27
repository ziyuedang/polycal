"""polycal — online LiDAR-camera extrinsic calibration with calibrated uncertainty."""

from __future__ import annotations

from polycal.ekf import ExtrinsicEKF
from polycal.lie_utils import se3_adjoint, se3_exp, se3_log
from polycal.metrics import interval_score, mpiw, picp
from polycal.synthetic import (
    CameraModel,
    Correspondence,
    LidarModel,
    Landmark,
    LinearDrift,
    LineSegment,
    OdometryConfig,
    Plane,
    SceneConfig,
    StaticDrift,
    SyntheticConfig,
    SyntheticDataset,
    VehicleTrajectoryConfig,
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
    "OdometryConfig",
    "VehicleTrajectoryConfig",
    "SyntheticConfig",
    "SyntheticDataset",
    "Correspondence",
    "generate",
    "ExtrinsicEKF",
    "se3_exp",
    "se3_log",
    "se3_adjoint",
    "picp",
    "mpiw",
    "interval_score",
    "__version__",
]

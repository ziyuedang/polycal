"""polycal — online LiDAR-camera extrinsic calibration with calibrated uncertainty."""

from __future__ import annotations

from polycal.cusum import (
    CUSUMCalibration,
    CUSUMConfig,
    CUSUMDetector,
    calibrate_kappa,
    compute_innovation_cov,
)
from polycal.ekf import ExtrinsicEKF
from polycal.lie_utils import (
    Q_left,
    se3_adjoint,
    se3_exp,
    se3_log,
    se3_right_jacobian_inverse,
    so3_right_jacobian,
    so3_right_jacobian_inverse,
)
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
    "CUSUMConfig",
    "CUSUMDetector",
    "CUSUMCalibration",
    "calibrate_kappa",
    "compute_innovation_cov",
    "se3_exp",
    "se3_log",
    "se3_adjoint",
    "so3_right_jacobian",
    "so3_right_jacobian_inverse",
    "Q_left",
    "se3_right_jacobian_inverse",
    "picp",
    "mpiw",
    "interval_score",
    "__version__",
]

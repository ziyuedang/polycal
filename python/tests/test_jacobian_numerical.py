from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from polycal.lie_utils import se3_adjoint, se3_exp, se3_log


def residual(
    T_lc: np.ndarray,
    T_cam_odom: np.ndarray,
    T_lidar_odom: np.ndarray,
    delta: np.ndarray,
) -> np.ndarray:
    """Compute perturbed hand-eye residual under right/local perturbation."""
    T_lc_p = T_lc @ se3_exp(delta)
    predicted = T_lc_p @ T_lidar_odom @ np.linalg.inv(T_lc_p)
    error = np.linalg.inv(T_cam_odom) @ predicted
    return se3_log(error)


def analytical_jacobian(T_lc: np.ndarray, T_lidar_odom: np.ndarray) -> np.ndarray:
    """Compute Phase 1 analytical Jacobian with J_r^{-1} approximated as identity."""
    return se3_adjoint(T_lc) @ (
        se3_adjoint(np.linalg.inv(T_lidar_odom)) - np.eye(6)
    )


def make_random_se3(rng: np.random.Generator) -> np.ndarray:
    """Random valid SE3 matrix."""
    rotvec = rng.normal(0, 0.3, 3)
    t = rng.normal(0, 0.5, 3)
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
    T[:3, 3] = t
    return T


def numerical_jacobian(
    T_lc: np.ndarray,
    T_cam_odom: np.ndarray,
    T_lidar_odom: np.ndarray,
    eps: float = 1e-6,
) -> np.ndarray:
    """Compute central-difference Jacobian of residual wrt right perturbation."""
    H = np.zeros((6, 6))
    for i in range(6):
        delta_plus = np.zeros(6)
        delta_plus[i] = eps
        delta_minus = np.zeros(6)
        delta_minus[i] = -eps
        H[:, i] = (
            residual(T_lc, T_cam_odom, T_lidar_odom, delta_plus)
            - residual(T_lc, T_cam_odom, T_lidar_odom, delta_minus)
        ) / (2 * eps)
    return H


def _assert_jacobian_matches(seed: int) -> None:
    rng = np.random.default_rng(seed)
    T_lc = make_random_se3(rng)
    T_lidar_odom = make_random_se3(rng)
    T_cam_odom = T_lc @ T_lidar_odom @ np.linalg.inv(T_lc)
    H_num = numerical_jacobian(T_lc, T_cam_odom, T_lidar_odom)
    H_ana = analytical_jacobian(T_lc, T_lidar_odom)
    if not np.allclose(H_num, H_ana, atol=1e-4):
        print("Numerical:\n", H_num)
        print("Analytical:\n", H_ana)
        print("Difference:\n", H_num - H_ana)
    assert np.allclose(H_num, H_ana, atol=1e-4)


def test_jacobian_config_1() -> None:
    _assert_jacobian_matches(42)


def test_jacobian_config_2() -> None:
    _assert_jacobian_matches(99)


def test_jacobian_config_3() -> None:
    _assert_jacobian_matches(7)

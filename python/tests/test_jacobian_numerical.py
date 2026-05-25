from __future__ import annotations

import numpy as np
import pytest
from scipy.spatial.transform import Rotation


def se3_exp(xi: np.ndarray) -> np.ndarray:
    """6D tangent vector [rho, phi] -> 4x4 SE3 matrix. Right/local convention."""
    rho = xi[:3]
    phi = xi[3:]
    angle = np.linalg.norm(phi)
    if angle < 1e-10:
        R = np.eye(3)
        V = np.eye(3)
    else:
        R = Rotation.from_rotvec(phi).as_matrix()
        phi_x = np.array(
            [
                [0, -phi[2], phi[1]],
                [phi[2], 0, -phi[0]],
                [-phi[1], phi[0], 0],
            ]
        )
        V = (
            np.eye(3)
            + (1 - np.cos(angle)) / angle**2 * phi_x
            + (angle - np.sin(angle)) / angle**3 * phi_x @ phi_x
        )
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = V @ rho
    return T


def se3_log(T: np.ndarray) -> np.ndarray:
    """4x4 SE3 matrix -> 6D [rho, phi] tangent vector."""
    R = T[:3, :3]
    t = T[:3, 3]
    phi = Rotation.from_matrix(R).as_rotvec()
    angle = np.linalg.norm(phi)
    if angle < 1e-10:
        V_inv = np.eye(3)
    else:
        phi_x = np.array(
            [
                [0, -phi[2], phi[1]],
                [phi[2], 0, -phi[0]],
                [-phi[1], phi[0], 0],
            ]
        )
        V_inv = (
            np.eye(3)
            - 0.5 * phi_x
            + (1 - angle * np.cos(angle / 2) / (2 * np.sin(angle / 2)))
            / angle**2
            * phi_x
            @ phi_x
        )
    rho = V_inv @ t
    return np.concatenate([rho, phi])


def se3_adjoint(T: np.ndarray) -> np.ndarray:
    """SE3 adjoint matrix, 6x6."""
    R = T[:3, :3]
    t = T[:3, 3]
    skew_t = np.array(
        [
            [0, -t[2], t[1]],
            [t[2], 0, -t[0]],
            [-t[1], t[0], 0],
        ]
    )
    adj = np.zeros((6, 6))
    adj[:3, :3] = R
    adj[:3, 3:] = skew_t @ R
    adj[3:, 3:] = R
    return adj


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

"""SE(3) Lie group helpers for Python-side estimator tests."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def skew(v: np.ndarray) -> np.ndarray:
    """3D skew-symmetric matrix of vector ``v``."""
    vector = np.asarray(v, dtype=float)
    if vector.shape != (3,):
        raise ValueError(f"v must have shape (3,), got {vector.shape}")
    return np.array(
        [
            [0.0, -vector[2], vector[1]],
            [vector[2], 0.0, -vector[0]],
            [-vector[1], vector[0], 0.0],
        ]
    )


def so3_right_jacobian(phi: np.ndarray) -> np.ndarray:
    """SO(3) right Jacobian ``J_r(phi)``."""
    rotation = np.asarray(phi, dtype=float)
    if rotation.shape != (3,):
        raise ValueError(f"phi must have shape (3,), got {rotation.shape}")
    angle = np.linalg.norm(rotation)
    if angle < 1e-7:
        return np.eye(3)
    phi_x = skew(rotation)
    return (
        np.eye(3)
        - (1.0 - np.cos(angle)) / angle**2 * phi_x
        + (angle - np.sin(angle)) / angle**3 * phi_x @ phi_x
    )


def so3_right_jacobian_inverse(phi: np.ndarray) -> np.ndarray:
    """SO(3) right Jacobian inverse ``J_r(phi)^{-1}``."""
    rotation = np.asarray(phi, dtype=float)
    if rotation.shape != (3,):
        raise ValueError(f"phi must have shape (3,), got {rotation.shape}")
    angle = np.linalg.norm(rotation)
    if angle < 1e-7:
        return np.eye(3)
    phi_x = skew(rotation)
    return (
        np.eye(3)
        + 0.5 * phi_x
        + (
            1.0 / angle**2
            - (1.0 + np.cos(angle)) / (2.0 * angle * np.sin(angle))
        )
        * phi_x
        @ phi_x
    )


def Q_left(rho: np.ndarray, phi: np.ndarray) -> np.ndarray:
    """SE(3) left ``Q`` matrix from Sola et al. arXiv:1812.01537 eq. 180."""
    translation = np.asarray(rho, dtype=float)
    rotation = np.asarray(phi, dtype=float)
    if translation.shape != (3,):
        raise ValueError(f"rho must have shape (3,), got {translation.shape}")
    if rotation.shape != (3,):
        raise ValueError(f"phi must have shape (3,), got {rotation.shape}")
    angle = np.linalg.norm(rotation)
    rho_x = skew(translation)
    if angle < 1e-7:
        return 0.5 * rho_x

    phi_x = skew(rotation)
    phi_x2 = phi_x @ phi_x
    return (
        0.5 * rho_x
        + (angle - np.sin(angle))
        / angle**3
        * (phi_x @ rho_x + rho_x @ phi_x + phi_x @ rho_x @ phi_x)
        - (1.0 - angle**2 / 2.0 - np.cos(angle))
        / angle**4
        * (phi_x2 @ rho_x + rho_x @ phi_x2 - 3.0 * phi_x @ rho_x @ phi_x)
        + (
            -0.5
            + (angle - np.sin(angle)) / angle**3
            - (angle**2 - 2.0 + 2.0 * np.cos(angle)) / (2.0 * angle**4)
        )
        * (phi_x @ rho_x @ phi_x2 + phi_x2 @ rho_x @ phi_x)
    )


def se3_right_jacobian_inverse(xi: np.ndarray) -> np.ndarray:
    """SE(3) right Jacobian inverse ``J_r(xi)^{-1}`` for ``xi = [rho, phi]``."""
    tangent = np.asarray(xi, dtype=float)
    if tangent.shape != (6,):
        raise ValueError(f"xi must have shape (6,), got {tangent.shape}")
    if np.linalg.norm(tangent) < 1e-7:
        return np.eye(6)

    rho = tangent[:3]
    phi = tangent[3:]
    Jr_inv = so3_right_jacobian_inverse(phi)
    Q_r = Q_left(-rho, -phi)
    result = np.eye(6)
    result[:3, :3] = Jr_inv
    result[:3, 3:] = -Jr_inv @ Q_r @ Jr_inv
    result[3:, 3:] = Jr_inv
    return result


def se3_exp(xi: np.ndarray) -> np.ndarray:
    """Convert a 6D tangent vector ``[rho, phi]`` to a 4x4 SE(3) matrix."""
    rho = np.asarray(xi[:3], dtype=float)
    phi = np.asarray(xi[3:], dtype=float)
    angle = np.linalg.norm(phi)
    if angle < 1e-10:
        rotation = np.eye(3)
        V = np.eye(3)
    else:
        rotation = Rotation.from_rotvec(phi).as_matrix()
        phi_x = np.array(
            [
                [0.0, -phi[2], phi[1]],
                [phi[2], 0.0, -phi[0]],
                [-phi[1], phi[0], 0.0],
            ]
        )
        V = (
            np.eye(3)
            + (1.0 - np.cos(angle)) / angle**2 * phi_x
            + (angle - np.sin(angle)) / angle**3 * phi_x @ phi_x
        )
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = V @ rho
    return T


def se3_log(T: np.ndarray) -> np.ndarray:
    """Convert a 4x4 SE(3) matrix to a 6D tangent vector ``[rho, phi]``."""
    rotation = np.asarray(T[:3, :3], dtype=float)
    translation = np.asarray(T[:3, 3], dtype=float)
    phi = Rotation.from_matrix(rotation).as_rotvec()
    angle = np.linalg.norm(phi)
    if angle < 1e-10:
        V_inv = np.eye(3)
    else:
        phi_x = np.array(
            [
                [0.0, -phi[2], phi[1]],
                [phi[2], 0.0, -phi[0]],
                [-phi[1], phi[0], 0.0],
            ]
        )
        V_inv = (
            np.eye(3)
            - 0.5 * phi_x
            + (1.0 - angle * np.cos(angle / 2.0) / (2.0 * np.sin(angle / 2.0)))
            / angle**2
            * phi_x
            @ phi_x
        )
    rho = V_inv @ translation
    return np.concatenate([rho, phi])


def se3_adjoint(T: np.ndarray) -> np.ndarray:
    """Return the 6x6 SE(3) adjoint matrix."""
    rotation = np.asarray(T[:3, :3], dtype=float)
    translation = np.asarray(T[:3, 3], dtype=float)
    skew_t = np.array(
        [
            [0.0, -translation[2], translation[1]],
            [translation[2], 0.0, -translation[0]],
            [-translation[1], translation[0], 0.0],
        ]
    )
    adjoint = np.zeros((6, 6))
    adjoint[:3, :3] = rotation
    adjoint[:3, 3:] = skew_t @ rotation
    adjoint[3:, 3:] = rotation
    return adjoint

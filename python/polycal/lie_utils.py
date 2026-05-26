"""SE(3) Lie group helpers for Python-side estimator tests."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


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

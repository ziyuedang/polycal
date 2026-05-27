from __future__ import annotations

import numpy as np

from polycal.lie_utils import (
    Q_left,
    se3_right_jacobian_inverse,
    skew,
    so3_right_jacobian,
    so3_right_jacobian_inverse,
)


def test_so3_jr_times_jr_inv_equals_identity() -> None:
    rng = np.random.default_rng(0)
    for _ in range(5):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        angle = rng.uniform(0.1, 1.0)
        phi = angle * axis

        J = so3_right_jacobian(phi)
        Jinv = so3_right_jacobian_inverse(phi)

        assert np.allclose(J @ Jinv, np.eye(3), atol=1e-10)
        assert np.allclose(Jinv @ J, np.eye(3), atol=1e-10)


def test_so3_jr_inv_small_angle() -> None:
    phi = np.array([1e-8, 1e-8, 1e-8])
    assert np.allclose(so3_right_jacobian_inverse(phi), np.eye(3), atol=1e-6)


def test_Q_left_small_angle() -> None:
    rho = np.array([0.3, -0.2, 0.1])
    phi = np.array([1e-8, 1e-8, 1e-8])
    Q = Q_left(rho, phi)
    expected = 0.5 * skew(rho)
    assert np.allclose(Q, expected, atol=1e-6)


def test_se3_jr_times_jr_inv_equals_identity() -> None:
    xi = np.array([0.3, -0.2, 0.1, 0.4, -0.3, 0.2])
    rho = xi[:3]
    phi = xi[3:]
    Jr_SO3 = so3_right_jacobian(phi)
    Q_r = Q_left(-rho, -phi)
    J = np.zeros((6, 6))
    J[:3, :3] = Jr_SO3
    J[:3, 3:] = Q_r
    J[3:, 3:] = Jr_SO3

    Jinv = se3_right_jacobian_inverse(xi)
    assert np.linalg.norm(J @ Jinv - np.eye(6)) < 1e-10


def test_se3_jr_inv_small_angle() -> None:
    xi = np.array([1e-8] * 6)
    assert np.allclose(se3_right_jacobian_inverse(xi), np.eye(6), atol=1e-6)

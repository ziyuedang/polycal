"""Pure Python mirror of the C++ extrinsic EKF scaffold."""

from __future__ import annotations

import numpy as np

from polycal.lie_utils import (
    se3_adjoint,
    se3_exp,
    se3_log,
    se3_right_jacobian_inverse,
)


class ExtrinsicEKF:
    """Motion-based hand-eye EKF matching ``cpp/src/ekf.cpp``."""

    def __init__(
        self,
        T_lc_init: np.ndarray,
        P_init: np.ndarray,
        Q: np.ndarray,
        backend: str = "python",
    ) -> None:
        """Initialize the EKF with an SE(3) extrinsic, covariance, and process noise.

        Args:
            T_lc_init: 4x4 SE3 initial extrinsic (LiDAR-to-camera).
            P_init: 6x6 initial covariance.
            Q: 6x6 process noise covariance rate.
            backend: ``"python"`` or ``"cpp"``.
        """
        if backend == "cpp":
            try:
                from polycal import _polycal_cpp
            except ImportError as e:
                raise ImportError(
                    "C++ backend not available. Build the cpp/ target and "
                    "ensure _polycal_cpp.so is in python/polycal/. "
                    f"Original error: {e}"
                ) from e
            self._backend = "cpp"
            self._cpp = _polycal_cpp.ExtrinsicEKF(T_lc_init, P_init, Q)
        elif backend == "python":
            self._backend = "python"
            self._cpp = None
            self.x_ = np.zeros(6, dtype=float)
            self.P_ = np.asarray(P_init, dtype=float).copy()
            self.Q_ = np.asarray(Q, dtype=float).copy()
            self.T_lc_ = np.asarray(T_lc_init, dtype=float).copy()
        else:
            raise ValueError(f"backend must be 'python' or 'cpp', got {backend!r}")

    def predict(self, dt: float) -> None:
        """Propagate covariance forward by ``dt`` seconds."""
        if self._backend == "cpp":
            self._cpp.predict(dt)
            return
        self.P_ += self.Q_ * dt

    def compute_residual(
        self,
        T_cam_odom: np.ndarray,
        T_lidar_odom: np.ndarray,
    ) -> np.ndarray:
        """Compute the hand-eye residual."""
        if self._backend == "cpp":
            return self._cpp.compute_residual(T_cam_odom, T_lidar_odom)
        predicted = self.T_lc_ @ T_lidar_odom @ np.linalg.inv(self.T_lc_)
        error = np.linalg.inv(T_cam_odom) @ predicted
        return se3_log(error)

    def compute_jacobian(
        self,
        T_cam_odom: np.ndarray,
        T_lidar_odom: np.ndarray,
    ) -> np.ndarray:
        """Compute the hand-eye Jacobian."""
        if self._backend == "cpp":
            raise NotImplementedError(
                "compute_jacobian not exposed in C++ backend. "
                "Use update() or update_with_cusum() instead."
            )
        predicted = self.T_lc_ @ T_lidar_odom @ np.linalg.inv(self.T_lc_)
        error = np.linalg.inv(T_cam_odom) @ predicted
        r = se3_log(error)

        # Closed-form J_r^{-1} per Sola et al. arXiv:1812.01537 eq. 179b
        Jr_inv = se3_right_jacobian_inverse(r)

        H_inside = se3_adjoint(self.T_lc_) @ (
            se3_adjoint(np.linalg.inv(T_lidar_odom)) - np.eye(6)
        )
        return Jr_inv @ H_inside

    def update(
        self,
        T_cam_odom: np.ndarray,
        T_lidar_odom: np.ndarray,
        R: np.ndarray,
    ) -> None:
        """Apply one EKF measurement update."""
        if self._backend == "cpp":
            self._cpp.update(T_cam_odom, T_lidar_odom, R)
            return
        H = self.compute_jacobian(T_cam_odom, T_lidar_odom)
        r = self.compute_residual(T_cam_odom, T_lidar_odom)
        S = H @ self.P_ @ H.T + R
        K = self.P_ @ H.T @ np.linalg.inv(S)
        self.x_ = self.x_ - K @ r
        self.P_ = (np.eye(6) - K @ H) @ self.P_
        self.T_lc_ = self.T_lc_ @ se3_exp(self.x_)
        self.x_ = np.zeros(6, dtype=float)

    def update_with_cusum(
        self,
        T_cam_odom: np.ndarray,
        T_lidar_odom: np.ndarray,
        R: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Apply one EKF update and return innovation data for CUSUM.

        Returns:
            Tuple ``(innovation, innovation_cov)`` where ``innovation`` is the
            residual vector and ``innovation_cov`` is ``S = H P H.T + R``.
        """
        if self._backend == "cpp":
            result = self._cpp.update_with_cusum(T_cam_odom, T_lidar_odom, R)
            return result.innovation, result.innovation_cov
        H = self.compute_jacobian(T_cam_odom, T_lidar_odom)
        r = self.compute_residual(T_cam_odom, T_lidar_odom)
        S = H @ self.P_ @ H.T + R
        K = self.P_ @ H.T @ np.linalg.inv(S)
        self.x_ = self.x_ - K @ r
        self.P_ = (np.eye(6) - K @ H) @ self.P_
        self.T_lc_ = self.T_lc_ @ se3_exp(self.x_)
        self.x_ = np.zeros(6, dtype=float)
        return r, S

    @property
    def T_lc(self) -> np.ndarray:
        """Return the current LiDAR-to-camera transform estimate."""
        if self._backend == "cpp":
            return self._cpp.T_lc()
        return self.T_lc_.copy()

    @property
    def P(self) -> np.ndarray:
        """Return the current covariance estimate."""
        if self._backend == "cpp":
            return self._cpp.P()
        return self.P_.copy()

    @property
    def x(self) -> np.ndarray:
        """Return the current local perturbation state."""
        if self._backend == "cpp":
            return self._cpp.x()
        return self.x_.copy()

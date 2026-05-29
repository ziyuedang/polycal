from __future__ import annotations

import numpy as np
import pytest

from polycal.cusum import CUSUMConfig, CUSUMDetector
from polycal.ekf import ExtrinsicEKF
from polycal.lie_utils import se3_exp

cpp_available = False
try:
    from polycal import _polycal_cpp  # noqa: F401

    cpp_available = True
except ImportError:
    pass

skip_if_no_cpp = pytest.mark.skipif(
    not cpp_available,
    reason="_polycal_cpp extension not built",
)


def _analytic_Q() -> np.ndarray:
    return np.diag([1e-8] * 3 + [1e-7] * 3)


def _analytic_R() -> np.ndarray:
    return np.diag([2e-4] * 3 + [2e-6] * 3)


def _make_odom(index: int) -> np.ndarray:
    tangent = np.zeros(6)
    value = float(index)
    tangent[0] = 0.2 + 0.01 * np.sin(0.1 * value)
    tangent[1] = -0.1 + 0.02 * np.cos(0.2 * value)
    tangent[3] = 0.05 * np.sin(0.05 * value)
    tangent[4] = -0.04 * np.cos(0.08 * value)
    tangent[5] = 0.1 * np.sin(0.03 * value)
    return se3_exp(tangent)


@skip_if_no_cpp
def test_ekf_predict_parity() -> None:
    P_init = 1e-4 * np.eye(6)
    Q = _analytic_Q()
    py_ekf = ExtrinsicEKF(np.eye(4), P_init, Q, backend="python")
    cpp_ekf = ExtrinsicEKF(np.eye(4), P_init, Q, backend="cpp")

    py_ekf.predict(0.1)
    cpp_ekf.predict(0.1)

    assert np.allclose(py_ekf.P, cpp_ekf.P, atol=1e-12)


@skip_if_no_cpp
def test_ekf_residual_parity() -> None:
    T_lc = se3_exp(np.array([0.03, -0.02, 0.01, 0.02, -0.01, 0.04]))
    T_lidar = _make_odom(3)
    T_cam = T_lc @ T_lidar @ np.linalg.inv(T_lc)
    py_ekf = ExtrinsicEKF(T_lc, 1e-4 * np.eye(6), _analytic_Q(), backend="python")
    cpp_ekf = ExtrinsicEKF(T_lc, 1e-4 * np.eye(6), _analytic_Q(), backend="cpp")

    py_residual = py_ekf.compute_residual(T_cam, T_lidar)
    cpp_residual = cpp_ekf.compute_residual(T_cam, T_lidar)

    assert np.allclose(py_residual, cpp_residual, atol=1e-10)


@skip_if_no_cpp
def test_ekf_update_parity() -> None:
    T_lc = se3_exp(np.array([0.03, -0.02, 0.01, 0.02, -0.01, 0.04]))
    P_init = 1e-4 * np.eye(6)
    Q = _analytic_Q()
    R = _analytic_R()
    py_ekf = ExtrinsicEKF(T_lc, P_init, Q, backend="python")
    cpp_ekf = ExtrinsicEKF(T_lc, P_init, Q, backend="cpp")

    for index in range(50):
        T_lidar = _make_odom(index)
        T_cam = T_lc @ T_lidar @ np.linalg.inv(T_lc)
        py_ekf.predict(0.1)
        cpp_ekf.predict(0.1)
        py_ekf.update(T_cam, T_lidar, R)
        cpp_ekf.update(T_cam, T_lidar, R)
        assert np.allclose(py_ekf.T_lc, cpp_ekf.T_lc, atol=1e-8)
        assert np.allclose(py_ekf.P, cpp_ekf.P, atol=1e-8)


@skip_if_no_cpp
def test_cusum_update_parity() -> None:
    rng = np.random.default_rng(0)
    config = CUSUMConfig()
    py_detector = CUSUMDetector(config, backend="python")
    cpp_detector = CUSUMDetector(config, backend="cpp")
    alarm_step_py = None
    alarm_step_cpp = None

    for step in range(1, 201):
        innovation = rng.standard_normal(6)
        if step > 100:
            innovation[3] += 3.0
        alarm_py, statistic_py = py_detector.update(innovation, np.eye(6))
        alarm_cpp, statistic_cpp = cpp_detector.update(innovation, np.eye(6))
        assert alarm_py == alarm_cpp
        assert np.isclose(statistic_py, statistic_cpp, atol=1e-10)
        if alarm_py and alarm_step_py is None:
            alarm_step_py = step
        if alarm_cpp and alarm_step_cpp is None:
            alarm_step_cpp = step

    assert alarm_step_py == alarm_step_cpp


@skip_if_no_cpp
def test_ekf_cusum_integration_parity() -> None:
    T_lc = se3_exp(np.array([0.03, -0.02, 0.01, 0.02, -0.01, 0.04]))
    P_init = 1e-4 * np.eye(6)
    Q = _analytic_Q()
    R = _analytic_R()
    py_ekf = ExtrinsicEKF(T_lc, P_init, Q, backend="python")
    cpp_ekf = ExtrinsicEKF(T_lc, P_init, Q, backend="cpp")
    py_detector = CUSUMDetector(CUSUMConfig(), backend="python")
    cpp_detector = CUSUMDetector(CUSUMConfig(), backend="cpp")
    py_trace = []
    cpp_trace = []
    py_alarm_step = None
    cpp_alarm_step = None

    for index in range(400):
        T_lidar = _make_odom(index)
        T_cam = T_lc @ T_lidar @ np.linalg.inv(T_lc)
        py_ekf.predict(0.1)
        cpp_ekf.predict(0.1)
        py_innovation, py_cov = py_ekf.update_with_cusum(T_cam, T_lidar, R)
        cpp_innovation, cpp_cov = cpp_ekf.update_with_cusum(T_cam, T_lidar, R)
        assert np.allclose(py_innovation, cpp_innovation, atol=1e-10)
        assert np.allclose(py_cov, cpp_cov, atol=1e-8)

        if index >= 200:
            py_innovation = py_innovation.copy()
            cpp_innovation = cpp_innovation.copy()
            py_innovation[5] += 3.0
            cpp_innovation[5] += 3.0

        alarm_py, statistic_py = py_detector.update(py_innovation, py_cov)
        alarm_cpp, statistic_cpp = cpp_detector.update(cpp_innovation, cpp_cov)
        py_trace.append(statistic_py)
        cpp_trace.append(statistic_cpp)
        assert alarm_py == alarm_cpp
        assert np.isclose(statistic_py, statistic_cpp, atol=1e-8)
        if alarm_py and py_alarm_step is None:
            py_alarm_step = index
        if alarm_cpp and cpp_alarm_step is None:
            cpp_alarm_step = index

    assert py_alarm_step == cpp_alarm_step
    assert np.allclose(py_trace, cpp_trace, atol=1e-8)

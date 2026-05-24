# polycal

Online LiDAR–camera extrinsic calibration with empirically calibrated
uncertainty and drift detection for autonomous vehicle fleets.

## What it is

polycal is a classical model-based library for continuous online estimation
of the 6-DOF extrinsic transform between a LiDAR and a camera, built in
three layers:

1. **Recursive estimator** — Extended Kalman Filter over SE(3), updated
   from streaming sensor observations with physical noise models and
   covariance propagation via Fisher information.
2. **Drift detection** — CUSUM change-point statistics on EKF innovations
   with controlled false-alarm rate and characterized detection delay.
   Flags when extrinsic parameters have shifted enough to require
   recalibration, without stopping the vehicle.
3. **Calibrated coverage** — stated x% confidence intervals empirically
   contain the true extrinsic x% of the time, validated against
   drift-injected synthetic ground truth.

## Why classical, not learning-based

Cocheteux et al. (WACV 2025, arXiv:2501.06878) showed that conformal
prediction can provide coverage-guaranteed intervals for online extrinsic
calibration using deep networks. polycal is the classical analog:

| | Cocheteux et al. | polycal |
|---|---|---|
| Uncertainty source | MC Dropout over neural network | Physical noise propagation via Fisher information |
| Training data required | Yes (60/15/15/10 labeled split) | No |
| Coverage semantics | Marginal (conformal) | Conditional (asymptotic, Fisher) |
| State | Stateless per-frame inference | Recursive EKF over time |
| Drift detection | Not implemented (named as future work §3) | Implemented with operating-characteristic curves |
| Compute | GPU, 25 forward passes per frame | CPU only, matrix operations |
| Operating range | ±1° / ±10cm (training distribution) | Bounded by EKF convergence basin |

polycal is also narrower in scope than OpenCalib (Yan et al.,
arXiv:2205.14087), which covers the full AV calibration matrix. polycal
does one thing: online LiDAR–camera extrinsic estimation with validated
uncertainty and drift detection.

For offline target-based calibration with rigorous covariance, see
Kogan's clc (github.com/dkogan/camera-lidar-calibration).

## Status

**Pre-alpha. Estimator not yet implemented.**

| Phase | Status | Contents |
|---|---|---|
| 0 — Evaluation harness | Complete | Synthetic data generator, PICP/MPIW/IS metrics, CI |
| 1 — EKF estimator | Next | Measurement noise model, SE(3) EKF, covariance output |
| 2 — Drift detection | Planned | CUSUM on innovations, false-alarm rate characterization |
| 3 — Coverage calibration | Planned | Empirical coverage validation, correction procedure |

## Installation

```bash
git clone https://github.com/ziyuedang/polycal
cd polycal
python3.11 -m venv .venv && source .venv/bin/activate
pip install -e ".[dev]"
pytest python/tests/ -v   # 22 passed
```

## References

Cocheteux, Moreau, Davoine. *Uncertainty-Aware Online Extrinsic
Calibration: A Conformal Prediction Approach.* WACV 2025.
arXiv:2501.06878.

Yan et al. *OpenCalib: A Multi-sensor Calibration Toolbox for Autonomous
Driving.* arXiv:2205.14087.

Basseville & Nikiforov. *Detection of Abrupt Changes in Signals and
Dynamical Systems.* 1993.

## License

Apache-2.0
